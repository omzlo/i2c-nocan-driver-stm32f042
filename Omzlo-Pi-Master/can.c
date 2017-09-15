#include "can.h"
#include <stm32f0xx_rcc.h>
#include <stm32f0xx_gpio.h>
#include <stm32f0xx_misc.h>
#include <stm32f0xx_can.h>
#include "gpio.h"

/***/

const can_configuration_t *can_conf;
const can_packet_t *can_tx_packet;

/***/

#define FILTER_COUNT 14
uint8_t filters[FILTER_COUNT];

int can_filter_modify(int filter_id, uint32_t can_id, uint32_t mask)
{
    CAN_FilterInitTypeDef CAN_FilterInitStructure;

    filters[filter_id] = 1;
    CAN_FilterInitStructure.CAN_FilterNumber = filter_id;
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh = (can_id >> 16);
    CAN_FilterInitStructure.CAN_FilterIdLow =  (can_id & 0xFFFF);
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = (mask >> 16);
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = (mask & 0xFFFF);
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);
    return filter_id;
}

int can_filter_add(uint32_t can_id, uint32_t mask)
{
    uint8_t i;

    for (i=0;i<FILTER_COUNT;i++)
    {
        if (filters[i]==0)
        {
            return can_filter_modify(i, can_id, mask);
        }
    }
    return -1;
}

int can_filter_remove(int filter_id)
{
    CAN_FilterInitTypeDef CAN_FilterInitStructure;

    CAN_FilterInitStructure.CAN_FilterNumber = filter_id;
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
    CAN_FilterInitStructure.CAN_FilterActivation = DISABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);
    filters[filter_id] = 0;
    return filter_id;
}

/********************/

static void load_packet(uint8_t fifo_number, can_packet_t *dest)
{
    dest->eid = (CAN->sFIFOMailBox[fifo_number].RIR>>3) | ((CAN->sFIFOMailBox[fifo_number].RIR & 0x4) << 29);
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
int can_send(const can_packet_t *packet) 
{
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

    CAN->sTxMailBox[transmit_mailbox].TIR = (packet->eid<<3) | (1<<2); 
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
    CAN_ITConfig(CAN, CAN_IT_TME, DISABLE); 
    gpio_set_CAN_TX_INT(0);
    CAN_ITConfig(CAN, CAN_IT_TME, ENABLE); 
    return 0;
}

/********************/
void CEC_CAN_IRQHandler(void)
{
    can_packet_t packet;

    if((CAN->RF0R & CAN_RF0R_FMP0)!=0)/* check if a packet is filtered and received by FIFO 0 */
    {   
        load_packet(0,&packet);
        can_conf->receive_fifo_0_cb(&packet);
        CAN->RF0R |= CAN_RF0R_RFOM0; // release the fifo
    }
    
    if((CAN->RF1R & CAN_RF1R_FMP1)!=0)/* check if a packet is filtered and received by FIFO 1 */
    {   
        load_packet(1,&packet);
        can_conf->receive_fifo_1_cb(&packet);
        CAN->RF1R |= CAN_RF1R_RFOM1; // release the fifo
    }

    if((CAN->TSR & CAN_TSR_RQCP0)!=0) /* check if request completed for mailbox 0 */
    {
        gpio_set_CAN_TX_INT(1);
        //can_conf->send_complete_0_cb();
        //can_tx_packet = 0;
        CAN->TSR |= CAN_TSR_RQCP0; // release flag
    } 
}

int can_init(const can_configuration_t *callbacks)
{
  GPIO_InitTypeDef      GPIO_InitStructure;
  NVIC_InitTypeDef      NVIC_InitStructure;
  CAN_InitTypeDef       CAN_InitStructure;
  
  can_conf = callbacks;

  /* REMAP PINS, Only for STM32F042 */
  // RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
  // SYSCFG->CFGR1 |= SYSCFG_CFGR1_PA11_PA12_RMP;

  /* CAN GPIOs configuration **************************************************/

  /* Enable GPIO clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

  /* Connect CAN pins to AF4 */
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_4);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_4); 
  
  /* Configure CAN RX and TX pins */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

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
  for (int i=0;i<FILTER_COUNT;i++) 
      can_filter_remove(i);
  can_filter_add(0,0);
  
  /* Enable FIFO 0 and FIFO 1 message pending Interrupts */
  CAN_ITConfig(CAN, CAN_IT_FMP0, ENABLE); 
  CAN_ITConfig(CAN, CAN_IT_FMP1, ENABLE); 
  CAN_ITConfig(CAN, CAN_IT_TME, ENABLE);
  return 0;
}


