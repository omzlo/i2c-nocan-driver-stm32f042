#include <stm32f0xx.h>
#include <stm32f0xx_rcc.h>
#include <stm32f0xx_gpio.h>

/*
 * RESET: PB8-BOOT0
 * CAN_INT: PA0
 * CAN_LED: PA6
 * VSENSE: PA4 (ADC)
 */

#define CAN_INT_Pin GPIO_Pin_0
#define CAN_LED_Pin GPIO_Pin_6
#define RESET_Pin GPIO_Pin_8

void gpio_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  // Initialize GPIO clock  
  // see stm32f0xx_rcc.h

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = CAN_INT_Pin | CAN_LED_Pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = RESET_Pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;    // Open Drain
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOB->BSRR = (1<<8); // Start
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void gpio_write_can_led(int val)
{
    if (val)
        GPIOA->BSRR = CAN_LED_Pin;
    else
        GPIOA->BRR = CAN_LED_Pin;
}

void gpio_write_can_int(int val)
{
    if (val)
        GPIOA->BSRR = CAN_INT_Pin;
    else
        GPIOA->BRR = CAN_INT_Pin;
}

void gpio_write_reset(int val)
{
    if (val)
        GPIOB->BSRR = RESET_Pin;
    else
        GPIOB->BRR = RESET_Pin;
}

