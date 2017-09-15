#include <stm32f0xx.h>
#include <stm32f0xx_rcc.h>
#include <stm32f0xx_gpio.h>
#include "usart.h"

/* NOTES
   - ON STM32F042 discovery board mini, led is on PB_3 
*/

void adc_init()
{
    /*** SET CLOCK FOR ADC ***/
    /* (1) Enable the peripheral clock of the ADC */
    /* (2) Start HSI14 RC oscillator */ 
    /* (3) Wait HSI14 is ready */
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; /* (1) */
    RCC->CR2 |= RCC_CR2_HSI14ON; /* (2) */
    while ((RCC->CR2 & RCC_CR2_HSI14RDY) == 0) /* (3) */
    {
        /* For robust implementation, add here time-out management */
    }  

    /*** CALIBRATE ADC ***/
    /* (1) Ensure that ADEN = 0 */
    /* (2) Clear ADEN */ 
    /* (3) Launch the calibration by setting ADCAL */
    /* (4) Wait until ADCAL=0 */
    if ((ADC1->CR & ADC_CR_ADEN) != 0) /* (1) */
    {
        ADC1->CR &= (uint32_t)(~ADC_CR_ADEN);  /* (2) */  
    }
    ADC1->CR |= ADC_CR_ADCAL; /* (3) */
    while ((ADC1->CR & ADC_CR_ADCAL) != 0) /* (4) */
    {
        /* For robust implementation, add here time-out management */
    }  


    /*** CONFIGURE GPIO PINS FOR ADC ***/
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;  // Enabe clock GPIOA
    GPIOA->MODER |= GPIO_MODER_MODER0;  // Analog mode on pin 0 (PA0)


    /*** ENABLE ADC ***/
    do 
    {
        /* For robust implementation, add here time-out management */
		ADC1->CR |= ADC_CR_ADEN; /* (1) */
    } while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) /* (2) */;

    /*** CONFIGURE CHANNELS FOR ADC ***/
    ADC1->CFGR2 &= ~ADC_CFGR2_CKMODE;  // Select HSI14 by writing 00 in CKMODE (reset value)
    ADC1->CFGR1 = 0;
    ADC1->CHSELR = ADC_CHSELR_CHSEL0    // Select ADC_IN0
        ;

    ADC1->SMPR |= ADC_SMPR_SMP_0 | ADC_SMPR_SMP_1 | ADC_SMPR_SMP_2; // 111 is 239.5 ADC clk 

}

uint16_t adc_get(void)
{
     ADC1->CR |= ADC_CR_ADSTART;
     while ((ADC1->ISR & ADC_ISR_EOC) == 0);
     //while ((ADC1->ISR & ADC_ISR_EOSEQ) == 0);
     //while ((ADC1->CR & ADC_CR_ADSTART) != 0);

     return ADC1->DR;
}


#define LED_PIN GPIO_Pin_3

void delay(void);

int main(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  int n = 0;

  // Initialize GPIO clock  
  // see stm32f0xx_rcc.h

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

  // Initialize LED pins
  // see stm32f0xx_gpio.h

  GPIO_StructInit(&GPIO_InitStructure);

  // Pin PB2

  GPIO_InitStructure.GPIO_Pin = LED_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /*

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
    */

  adc_init();

  usart_init(115200);

  usart_printf("Start\n");
  while(1)
  {
    usart_printf("Iter %u:", n);
    usart_printf("%u\n",adc_get());
    n++;
    delay();

    GPIO_WriteBit(GPIOB, LED_PIN, (n&1) ? Bit_SET : Bit_RESET);
    //GPIO_WriteBit(GPIOC, GPIO_Pin_9, (n&4) ? Bit_SET : Bit_RESET);
  }
}

void delay(void) 
{
  int i = 4800000;/* About 1/4 second delay */
  while (i-- > 0) {
    asm("nop");/* This stops it optimising code out */
  }
}

/*
 * Debug help -- if asserts are enabled, assertion failure
 * from standard peripheral  library ends up here 
 */


#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* Infinite loop */
  /* Use GDB to find out why we're here */
  while (1)
  {
  }
}
#endif

