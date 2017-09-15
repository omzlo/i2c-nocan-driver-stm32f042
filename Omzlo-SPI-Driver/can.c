#include "can.h"
#include <stm32f0xx.h>
#include "stm32f0_helpers.h"
#include "nocan.h"

uint32_t can_tx_eid;
const uint8_t *can_tx_buffer;
volatile uint32_t can_tx_head;
volatile uint32_t can_tx_tail;
volatile int can_tx_state;

#define TX_STATE_EMPTY      0
#define TX_STATE_PENDING    1

static int can_pub_filter_validate(uint32_t eid)
{
    return 1;
}

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

#define FILTER_COUNT 12
uint16_t filters[FILTER_COUNT];

int can_msg_filter_channel_add(uint16_t channelid)
{
    uint8_t i;

    for (i=0;i<FILTER_COUNT;i++)
    {
        if (filters[i]==0xFFFF)
        {
            can_filter_set(i, ((uint32_t)channelid<<3) | 0x0004, 0x0007FFF8 | 0x0004, FIFO_0);
            filters[i] = channelid;
            return 1;
        }
    }
    return 0;
}

int can_msg_filter_channel_remove(uint16_t channelid)
{
    uint8_t i;

    for (i=0;i<FILTER_COUNT;i++)
    {
        if (filters[i]==channelid)
        {
            can_filter_deactivate(i);
            filters[i] = 0xFFFF;
            return 1;
        }
    }
    return 0;
}


/********************/
#if 0
static void load_packet(uint8_t fifo_number, can_packet_t *dest)
{
    dest->eid = CAN->sFIFOMailBox[fifo_number].RIR>>3;
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
#endif
/********************/

int can_tx_empty(void)
{
    return can_tx_state == TX_STATE_EMPTY;
}

static int send_packet(void) 
{
    uint32_t eid = can_tx_eid;
    uint8_t dlc;
    #define transmit_mailbox 0

    if (can_tx_state!=TX_STATE_PENDING)
    {
        return -1;
    }

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

    if (can_tx_head==0)
        eid |= (1<<28);

    if (can_tx_tail - can_tx_head<=8)
    {
        eid |= (1<<20);
        dlc = can_tx_tail - can_tx_head;
        can_tx_state = TX_STATE_EMPTY;
    }
    else
    {
        dlc = 8;
        can_tx_state = TX_STATE_PENDING;
    }
    
    const uint8_t *data = can_tx_buffer + can_tx_head;

    CAN->sTxMailBox[transmit_mailbox].TIR = (eid<<3) | (1<<2); 
        // (1<<2) means extended id, eid is shifted left by 3

    CAN->sTxMailBox[transmit_mailbox].TDTR &= (uint32_t)0xFFFFFFF0;
    CAN->sTxMailBox[transmit_mailbox].TDTR |= dlc;

    if (data)
    {
        CAN->sTxMailBox[transmit_mailbox].TDLR = (((uint32_t)data[3] << 24) |
                ((uint32_t)data[2] << 16) |
                ((uint32_t)data[1] << 8) |
                ((uint32_t)data[0]));
        CAN->sTxMailBox[transmit_mailbox].TDHR = (((uint32_t)data[7] << 24) |
                ((uint32_t)data[6] << 16) |
                ((uint32_t)data[5] << 8) |
                ((uint32_t)data[4]));
    }
    can_tx_head += dlc;
    CAN->sTxMailBox[transmit_mailbox].TIR |= 1; // (1<<0) is TXRQ flag (transmit request)
    return 0;
}

int can_tx_send(uint32_t eid, const uint8_t *data, uint8_t dlen)
{
    if (can_tx_state!=TX_STATE_EMPTY)
        return 0;

    CAN->IER &= ~CAN_IER_TMEIE; // disable  Transmit mailbox empty interrupt
    can_tx_eid = eid;
    can_tx_head = 0;
    can_tx_tail = dlen;
    can_tx_buffer = data;
    can_tx_state = TX_STATE_PENDING;
    CAN->IER |= CAN_IER_TMEIE; // enable  Transmit mailbox empty interrupt

    // get the pump going
    return send_packet();
}



/********************/
void CEC_CAN_IRQHandler(void)
{
    can_packet_t packet;

    if((CAN->RF0R & CAN_RF0R_FMP0)!=0)/* check if a packet is filtered and received by FIFO 0 */
    {   
        if (can_pub_filter_validate(CAN->sFIFOMailBox[0].RIR>>3))
        {
            nocan_mux_process((can_packet_t *)CAN->sFIFOMailBox+0);
        }
        CAN->RF0R |= CAN_RF0R_RFOM0; // release the fifo
        NOCAN->CAN_RX_COUNT++;
    }
    
    if((CAN->RF1R & CAN_RF1R_FMP1)!=0)/* check if a packet is filtered and received by FIFO 1 */
    {   
        nocan_sys_process((can_packet_t *)CAN->sFIFOMailBox+1);
        CAN->RF1R |= CAN_RF1R_RFOM1; // release the fifo
        NOCAN->CAN_RX_COUNT++;
    }

    if((CAN->TSR & CAN_TSR_RQCP0)!=0) /* check if request completed for mailbox 0 */
    {
        CAN->TSR |= CAN_TSR_RQCP0; // release flag
        if (can_tx_state == TX_STATE_PENDING)
        {
            send_packet();
        }
        else
        {
            nocan_status_clear(NOCAN_STATUS_TX_PENDING); 
        }
        NOCAN->CAN_TX_COUNT++;
    } 
}

int can_init(void)
{
    /* REMAP PINS */

    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
    SYSCFG->CFGR1 |= SYSCFG_CFGR1_PA11_PA12_RMP;

    /* CAN GPIOs configuration **************************************************/

    /* Enable GPIO clock */
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    /* Connect CAN pins to AF4 */
    GPIO_CONFIGURE_ALTERNATE_FUNCTION(GPIOA, 11, GPIO_AF_4);
    GPIO_CONFIGURE_ALTERNATE_FUNCTION(GPIOA, 12, GPIO_AF_4);


    /* CAN configuration ********************************************************/  
    /* Enable CAN clock */
    RCC->APB1ENR |= RCC_APB1ENR_CANEN;

    /* CAN cell init */
    CAN->MCR &=~ CAN_MCR_SLEEP;  // Exit sleep mode
   
    /* CAN register init */
    CAN->MCR |= CAN_MCR_INRQ; /* (1) */
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
    for (int i=0;i<FILTER_COUNT;i++) 
    {
        can_filter_deactivate(i);
        filters[i] = 0xFFFF;
    }
    can_sys_filter_set(0);

    /* Enable FIFO 0 and FIFO 1 message pending Interrupts 
     * Enable transmit mailbox empty Interrupt is set in can_tx_send()
     */
    CAN->IER = CAN_IER_FMPIE0
        | CAN_IER_FMPIE1
        // | CAN_IER_TMEIE
        ;

    /* NVIC configuration *******************************************************/
    NVIC_SetPriority(CEC_CAN_IRQn, 1); 
    NVIC_EnableIRQ(CEC_CAN_IRQn);
}


