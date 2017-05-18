#include "can.h"
#include <stm32f0xx_rcc.h>
#include <stm32f0xx_gpio.h>
#include <stm32f0xx_misc.h>
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

int can_sys_filter_set(uint8_t node_id)
{
    CAN_FilterInitTypeDef CAN_FilterInitStructure;

    CAN_FilterInitStructure.CAN_FilterNumber = 13;
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    
    //CAN_FilterInitStructure.CAN_FilterIdHigh = 0;
    //CAN_FilterInitStructure.CAN_FilterIdLow = 0;   
    //CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0;
    //CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0;   // sys bit + eid bit
    
    CAN_FilterInitStructure.CAN_FilterIdHigh = (((uint16_t)node_id&0x7F)<<8) | 0x20;
    CAN_FilterInitStructure.CAN_FilterIdLow = 0x0004;       // eid bit
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x7F20;  // + sys bit
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0004;   // eid bit
    
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 1;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);

    return 1;
}

#define FILTER_COUNT 12
uint16_t filters[FILTER_COUNT];

int can_msg_filter_channel_add(uint16_t channelid)
{
    uint8_t i;
    CAN_FilterInitTypeDef CAN_FilterInitStructure;

    for (i=0;i<FILTER_COUNT;i++)
    {
        if (filters[i]==0xFFFF)
        {
            filters[i] = channelid;
            CAN_FilterInitStructure.CAN_FilterNumber = i;
            CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
            CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
            CAN_FilterInitStructure.CAN_FilterIdHigh = (channelid>>13);
            CAN_FilterInitStructure.CAN_FilterIdLow = (channelid<<3) | 0x0004;
            CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0007;
            CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0xFFF8 | 0x0004;
            CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
            CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
            CAN_FilterInit(&CAN_FilterInitStructure);
            return 1;
        }
    }
    return 0;
}

static int _can_filter_disable(int filter)
{
    CAN_FilterInitTypeDef CAN_FilterInitStructure;

    CAN_FilterInitStructure.CAN_FilterNumber = filter;
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
    CAN_FilterInitStructure.CAN_FilterActivation = DISABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);
    filters[filter] = 0xFFFF;
}

int can_msg_filter_channel_remove(uint16_t channelid)
{
    uint8_t i;

    for (i=0;i<FILTER_COUNT;i++)
    {
        if (filters[i]==channelid)
        {
            return _can_filter_disable(i);
        }
    }
    return 0;
}


/********************/

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

    CAN_ITConfig(CAN, CAN_IT_TME, DISABLE); 
    can_tx_eid = eid;
    can_tx_head = 0;
    can_tx_tail = dlen;
    can_tx_buffer = data;
    can_tx_state = TX_STATE_PENDING;
    CAN_ITConfig(CAN, CAN_IT_TME, ENABLE); 

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
            load_packet(0,&packet);
            nocan_mux_process(&packet);
        }
        CAN->RF0R |= CAN_RF0R_RFOM0; // release the fifo
        NOCAN->CAN_RX_COUNT++;
    }
    
    if((CAN->RF1R & CAN_RF1R_FMP1)!=0)/* check if a packet is filtered and received by FIFO 1 */
    {   
        load_packet(1,&packet);
        nocan_sys_process(&packet);
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
            nocan_status_set(NOCAN_STATUS_TX_RDY); 
        }
        NOCAN->CAN_TX_COUNT++;
    } 
}

int can_init(void)
{
  GPIO_InitTypeDef      GPIO_InitStructure;
  NVIC_InitTypeDef      NVIC_InitStructure;
  CAN_InitTypeDef       CAN_InitStructure;
  CAN_FilterInitTypeDef CAN_FilterInitStructure;
  
  /* REMAP PINS */

  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
  SYSCFG->CFGR1 |= SYSCFG_CFGR1_PA11_PA12_RMP;

  /* CAN GPIOs configuration **************************************************/

  /* Enable GPIO clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

  /* Connect CAN pins to AF4 */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_4);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_4); 
  
  /* Configure CAN RX and TX pins */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* NVIC configuration *******************************************************/
  NVIC_InitStructure.NVIC_IRQChannel = CEC_CAN_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0x0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* CAN configuration ********************************************************/  
  /* Enable CAN clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN, ENABLE);
  
  /* CAN register init */
  CAN_DeInit(CAN);
  CAN_StructInit(&CAN_InitStructure);

  /* CAN cell init */
  CAN_InitStructure.CAN_TTCM = DISABLE;
  //CAN_InitStructure.CAN_ABOM = DISABLE;
  CAN_InitStructure.CAN_ABOM = ENABLE;
  CAN_InitStructure.CAN_AWUM = DISABLE;
  CAN_InitStructure.CAN_NART = DISABLE;
  CAN_InitStructure.CAN_RFLM = DISABLE;
  CAN_InitStructure.CAN_TXFP = DISABLE;
  CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
  CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
    
  /* CAN Baudrate = 125k (CAN clocked at 48 MHz) */
  // (48000000/125000)/24 => 16 => 1+BS1+BS2 (1+BS1)/16==75%
  CAN_InitStructure.CAN_BS1 = CAN_BS1_11tq;
  CAN_InitStructure.CAN_BS2 = CAN_BS2_4tq;
  CAN_InitStructure.CAN_Prescaler = 24;
  CAN_Init(CAN, &CAN_InitStructure);

  /* CAN filter init */
  for (int i=0;i<FILTER_COUNT;i++) _can_filter_disable(i);
  can_sys_filter_set(0);
  
  /* Enable FIFO 0 and FIFO 1 message pending Interrupts */
  CAN_ITConfig(CAN, CAN_IT_FMP0, ENABLE); 
  CAN_ITConfig(CAN, CAN_IT_FMP1, ENABLE); 
}


