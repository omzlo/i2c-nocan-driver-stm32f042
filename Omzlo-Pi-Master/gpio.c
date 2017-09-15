#include <stm32f0xx.h>
#include <stm32f0xx_rcc.h>
#include <stm32f0xx_gpio.h>

/*
 * CAN_RX_INT: PB10
 * CAN_TX_INT: PB11
 * LED_YEL: PB3 
 * LED_RED: PB2
 * PWR_IN: PB0
 * PWR_DEN: PB1
 */

#define PWR_IN_Pin      GPIO_Pin_0
#define PWR_DEN_Pin     GPIO_Pin_1
#define LED_RED_Pin     GPIO_Pin_2
#define LED_YEL_Pin     GPIO_Pin_3
#define CAN_RX_INT_Pin  GPIO_Pin_10
#define CAN_TX_INT_Pin  GPIO_Pin_11

void gpio_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  // Initialize GPIO clock  
  // see stm32f0xx_rcc.h

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = 
      PWR_IN_Pin |
      PWR_DEN_Pin |
      LED_RED_Pin |
      LED_YEL_Pin |
      CAN_RX_INT_Pin |
      CAN_TX_INT_Pin
      ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

}

#define gpio_set_B(pin, val) do if (val) GPIOB->BSRR = (pin); else GPIOB->BRR = (pin); while(0)

void gpio_set_LED_YEL(int val)
{
    gpio_set_B(LED_YEL_Pin, val);
}

void gpio_set_LED_RED(int val)
{
    gpio_set_B(LED_RED_Pin, val);
}

void gpio_set_CAN_RX_INT(int val)
{
    gpio_set_B(CAN_RX_INT_Pin,val);
}

void gpio_set_CAN_TX_INT(int val)
{
    gpio_set_B(CAN_TX_INT_Pin,val);
}

void gpio_set_PWR_IN(int val)
{
    gpio_set_B(PWR_IN_Pin,val);
}

void gpio_set_PWR_DEN(int val)
{
    gpio_set_B(PWR_DEN_Pin,val);
}

