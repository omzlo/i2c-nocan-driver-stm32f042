#include <stm32f0xx.h>
#include "stm32f0_helpers.h"

/*
 * CAN_RX_INT: PA0
 * CAN_TX_INT: PA1
 * LED: PB1
 * RESET: PB8-BOOT0
 */

#define CAN_RX_INT_Pin 0    
#define CAN_TX_INT_Pin 1
#define CAN_LED_Pin 6
#define RESET_Pin 8

void gpio_init(void)
{
  // enable clock on GPIOA
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

  // Configure pins on GPIOA
  GPIO_CONFIGURE_OUTPUT(GPIOA, CAN_TX_INT_Pin, GPIO_SPEED_LOW, GPIO_PUSH_PULL);
  GPIO_CONFIGURE_OUTPUT(GPIOA, CAN_RX_INT_Pin, GPIO_SPEED_LOW, GPIO_PUSH_PULL);
 
  // enable clock on GPIOB
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

  // configure pins on GPIOB
  GPIO_CONFIGURE_OUTPUT(GPIOB, CAN_LED_Pin, GPIO_SPEED_LOW, GPIO_PUSH_PULL);
  GPIO_CONFIGURE_OUTPUT(GPIOB, RESET_Pin, GPIO_SPEED_LOW, GPIO_OPEN_DRAIN);
  
  // set it to high
  GPIOB->BSRR = (1<<RESET_Pin);
}

void gpio_write_led(int val)
{
    if (val)
        GPIOB->BSRR = CAN_LED_Pin;
    else
        GPIOB->BRR = CAN_LED_Pin;
}

void gpio_write_can_tx_int(int val)
{
    if (val)
        GPIOA->BSRR = CAN_TX_INT_Pin;
    else
        GPIOA->BRR = CAN_TX_INT_Pin;
}

void gpio_write_can_rx_int(int val)
{
    if (val)
        GPIOA->BSRR = CAN_RX_INT_Pin;
    else
        GPIOA->BRR = CAN_RX_INT_Pin;
}

void gpio_write_reset(int val)
{
    if (val)
        GPIOB->BSRR = RESET_Pin;
    else
        GPIOB->BRR = RESET_Pin;
}


