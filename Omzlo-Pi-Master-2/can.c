#include "can.h"
#include <stm32f0xx.h>
#include <stm32f0_helpers.h>
#include "registers.h"
#include "gpio.h"

/***/

#define FILTER_COUNT 14
uint16_t filters[FILTER_COUNT];

#define FIFO_0 0
#define FIFO_1 1

void can_filter_set(uint32_t filter_id, uint32_t filter, uint32_t mask, int fifo)
{
    // Init mode
    CAN->FMR |= CAN_FMR_FINIT;

    // Deactivate filter
    CAN->FA1R &= ~(1<<filter_id);

    // 32 bitr scale for filter
    CAN->FS1R |= (1<<filter_id);

    CAN->sFilterRegister[filter_id].FR1 = filter;
    CAN->sFilterRegister[filter_id].FR2 = mask;
    
    // Id/Mask mode (0)
    CAN->FM1R &= ~(1<<filter_id);

    // set fifo
    if (fifo==FIFO_0)
        CAN->FFA1R &= ~(1<<filter_id);
    if (fifo==FIFO_1)
        CAN->FFA1R |= (1<<filter_id);

    // activate
    CAN->FA1R |= (1<<filter_id);
    
    // Leave filter init mode
    CAN->FMR &= ~CAN_FMR_FINIT;
}

void can_filter_deactivate(uint32_t filter_id)
{
    // Init mode
    CAN->FMR |= CAN_FMR_FINIT;

    // Deactivate filter
    CAN->FA1R &= ~(1<<filter_id);

    // Leave filter init mode
    CAN->FMR &= ~CAN_FMR_FINIT;
}

int can_sys_filter_set(uint8_t node_id)
{
    can_filter_set(13, ((uint32_t)node_id&0x7F)<<24 | 0x200004, 0x7F200004, FIFO_1);
            // 0x200000 is sys bit, 0x04 is eid bit
    return 1;
}


/********************/

static void load_packet(uint8_t fifo_number, can_packet_typedef *dest)
{
    uint32_t eid = (CAN->sFIFOMailBox[fifo_number].RIR>>3) | ((CAN->sFIFOMailBox[fifo_number].RIR & 0x4) << 29);
    
    dest->eid[0] = (uint8_t)(eid>>24) & 0xFF;
    dest->eid[1] = (uint8_t)(eid>>16) & 0xFF;
    dest->eid[2] = (uint8_t)(eid>>8) & 0xFF;
    dest->eid[3] = (uint8_t)(eid) & 0xFF;

    dest->dlc = (uint8_t)CAN->sFIFOMailBox[fifo_number].RDTR & 0xF;
    
    dest->data[0] = (uint8_t)CAN->sFIFOMailBox[fifo_number].RDLR & 0xFF;
    dest->data[1] = (uint8_t)(CAN->sFIFOMailBox[fifo_number].RDLR >>8)& 0xFF;
    dest->data[2] = (uint8_t)(CAN->sFIFOMailBox[fifo_number].RDLR >>16)& 0xFF;
    dest->data[3] = (uint8_t)(CAN->sFIFOMailBox[fifo_number].RDLR >>24)& 0xFF;
    dest->data[4] = (uint8_t)CAN->sFIFOMailBox[fifo_number].RDHR & 0xFF;
    dest->data[5] = (uint8_t)(CAN->sFIFOMailBox[fifo_number].RDHR >> 8) & 0xFF;
    dest->data[6] = (uint8_t)(CAN->sFIFOMailBox[fifo_number].RDHR >> 16)& 0xFF;
    dest->data[7] = (uint8_t)(CAN->sFIFOMailBox[fifo_number].RDHR >> 24)& 0xFF; 
}

/********************/
/*
int can_tx_ready(void)
{
    return can_tx_packet == 0;
}
*/
int can_send(const can_packet_typedef *packet) 
{
    uint32_t eid; 
    #define transmit_mailbox 0

//    if (!can_tx_ready())
//        return -1;
    

    /*
    if ((CANx->TSR&CAN_TSR_TME0) == CAN_TSR_TME0)
    {
        transmit_mailbox = 0;
    }
    else if ((CANx->TSR&CAN_TSR_TME1) == CAN_TSR_TME1)
    {
        transmit_mailbox = 1;
    }
    else if ((CANx->TSR&CAN_TSR_TME2) == CAN_TSR_TME2)
    {
        transmit_mailbox = 2;
    }
    else
    {
        return -1;
    }
    */

    // check if transmit mailbox 0 is avaiable
    if ((CAN->TSR&CAN_TSR_TME0) != CAN_TSR_TME0)
        return -1;

    eid = 
        ((uint32_t)packet->eid[0]<<24) |
        ((uint32_t)packet->eid[1]<<16) |
        ((uint32_t)packet->eid[2]<<8) |
        ((uint32_t)packet->eid[3]);


    CAN->sTxMailBox[transmit_mailbox].TIR = (eid<<3) | (1<<2); 
        // (1<<2) means extended id, eid is shifted left by 3

    CAN->sTxMailBox[transmit_mailbox].TDTR &= (uint32_t)0xFFFFFFF0;
    CAN->sTxMailBox[transmit_mailbox].TDTR |= packet->dlc;

    CAN->sTxMailBox[transmit_mailbox].TDLR = (
            ((uint32_t)packet->data[3] << 24) |
            ((uint32_t)packet->data[2] << 16) |
            ((uint32_t)packet->data[1] << 8) |
            ((uint32_t)packet->data[0]));
    CAN->sTxMailBox[transmit_mailbox].TDHR = (
            ((uint32_t)packet->data[7] << 24) |
            ((uint32_t)packet->data[6] << 16) |
            ((uint32_t)packet->data[5] << 8) |
            ((uint32_t)packet->data[4]));
    CAN->sTxMailBox[transmit_mailbox].TIR |= 1; // (1<<0) is TXRQ flag (transmit request)


    // there is a super tricky race condition here, so we need this:
    //CAN_ITConfig(CAN, CAN_IT_TME, DISABLE); 
    //gpio_set_CAN_TX_INT(0);
    //CAN_ITConfig(CAN, CAN_IT_TME, ENABLE); 
    return 0;
}

/********************/
void CEC_CAN_IRQHandler(void)
{
    can_packet_typedef *packet;

    if((CAN->RF0R & CAN_RF0R_FMP0)!=0)/* check if a packet is filtered and received by FIFO 0 */
    {  
        if ((packet = registers_recv_back())!=0)
        {
            load_packet(0,packet);
            registers_recv_push_back();
            CAN->RF0R |= CAN_RF0R_RFOM0; // release the fifo
        }
    }
    
    if((CAN->RF1R & CAN_RF1R_FMP1)!=0)/* check if a packet is filtered and received by FIFO 1 */
    {   
        if ((packet = registers_recv_back())!=0)
        {
            load_packet(1,packet);
            registers_recv_push_back();
            CAN->RF1R |= CAN_RF1R_RFOM1; // release the fifo
        }
    }

    if((CAN->TSR & CAN_TSR_RQCP0)!=0) /* check if request completed for mailbox 0 */
    {
        registers_send_done();
        CAN->TSR |= CAN_TSR_RQCP0; // release flag
    }
}

int can_init(void)
{
    /* Enable GPIO clock */
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    /* Connect CAN pins to AF4 */
    GPIO_CONFIGURE_ALTERNATE_FUNCTION(GPIOB, 8, GPIO_AF_4);
    GPIO_CONFIGURE_ALTERNATE_FUNCTION(GPIOB, 9, GPIO_AF_4);


    /* CAN configuration ********************************************************/
    /* Enable CAN clock */
    RCC->APB1ENR |= RCC_APB1ENR_CANEN;

    /* CAN register init */
    CAN->MCR &= ~CAN_MCR_SLEEP;  // Exit sleep mode

    /* CAN register init */
    CAN->MCR |= CAN_MCR_INRQ; /* Enter init mode */
    while((CAN->MSR & CAN_MSR_INAK)!=CAN_MSR_INAK) {}


    /* Automatic bus off management: 
     * The Bus-Off state is left automatically by hardware once 128 occurrences of 11 recessive
     * bits have been monitored.
     */
    CAN->MCR |= CAN_MCR_ABOM;

#define CAN_SJW_1tq 0
#define CAN_BS1_11tq 10
#define CAN_BS2_4tq 3 

    /* CAN Baudrate = 125k (CAN clocked at 48 MHz) */
    // (48000000/125000)/24 => 16 => 1+BS1+BS2 (1+BS1)/16==75%
    CAN->BTR = (0 << 30) // normal mode
        | (CAN_SJW_1tq << 24)
        | (CAN_BS1_11tq << 16)
        | (CAN_BS2_4tq << 20)
        | (24-1)              // prescaler
        ;

    /* Leave Init mode */
    CAN->MCR &= ~CAN_MCR_INRQ;
    while((CAN->MSR & CAN_MSR_INAK) == CAN_MSR_INAK) {}

    /* CAN filter init */
    //for (int i=0;i<FILTER_COUNT;i++) 
    //{
    //    can_filter_deactivate(i);
    //    filters[i] = 0xFFFF;
    //}
    //can_sys_filter_set(0);
    can_filter_set(0, 0, 0, FIFO_0);

    /* Enable FIFO 0 and FIFO 1 message pending Interrupts */
    CAN->IER = CAN_IER_FMPIE0
        | CAN_IER_FMPIE1
        | CAN_IER_TMEIE
        ;

    /* NVIC configuration *******************************************************/
    NVIC_SetPriority(CEC_CAN_IRQn, 1); 
    NVIC_EnableIRQ(CEC_CAN_IRQn);

    // Set default pin state
    gpio_set_CAN_RX_INT(HIGH);
    gpio_set_CAN_TX_INT(HIGH);
    return 0;
}


