#include "adc.h"
#include "stm32f0xx.h"
#include "gpio.h"


/*
 * Analog SENSE is on PA3/ADC_IN3
 * Analog VIN/11 is on PA2/ADC_IN2
 * Temp sensor is on ADC_IN16
 * VREF sensor is on ADC_IN17
 */

uint32_t CurrentChannel;
uint16_t *ChannelReadings;

void adc_init(uint16_t *channel_readings)
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
    GPIOA->MODER |= GPIO_MODER_MODER2;  // Analog mode on pin 2 (PA2)
    GPIOA->MODER |= GPIO_MODER_MODER3;  // Analog mode on pin 3 (PA3)


    /*** ENABLE ADC ***/
    do 
    {
        /* For robust implementation, add here time-out management */
		ADC1->CR |= ADC_CR_ADEN; /* (1) */
    } while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) /* (2) */;

    /*** CONFIGURE CHANNELS FOR ADC ***/
    //ADC1->CFGR2 &= ~ADC_CFGR2_CKMODE;  // Select HSI14 by writing 00 in CKMODE (reset value)
    ADC1->CFGR1 |= ADC_CFGR1_EXTEN_0 // 2^0 -> on rising edge (EXTEN=01) 
        | ADC_CFGR1_EXTSEL_2         // 2^2 -> TRG4, source is TIM15_TRGO
        ;                            // Setting ADC_CFGR1_SCANDIR would result in channels scanning from top to bottom
    ADC1->CHSELR = ADC_CHSELR_CHSEL2    // Select ADC_IN2
        | ADC_CHSELR_CHSEL3             // Select ADC_IN3
        | ADC_CHSELR_CHSEL17            // Select ADC_IN17
        ;
    ADC1->SMPR |= ADC_SMPR_SMP_0 | ADC_SMPR_SMP_1 | ADC_SMPR_SMP_2; // 111 is 239.5 ADC clk 
    ADC1->IER = ADC_IER_EOCIE       // Interrupt on EndOfConvertion (EOC)
        | ADC_IER_EOSEQIE           // Interrupt on EndOfSequence (EOSEQ)
        | ADC_IER_OVRIE             // Interrupt on Overflow
        ;
    ADC->CCR |= ADC_CCR_VREFEN;     // Enable internal voltage reference a.k.a. ADC_IN17

    /*** Enable ADC interrupt handler ***/
    NVIC_EnableIRQ(ADC1_COMP_IRQn); /* (7) */
    NVIC_SetPriority(ADC1_COMP_IRQn,1); /* (8) */

    /*** CONFIGURE TIM15 ***/
    /* (1) Enable the peripheral clock of the TIM15 */ 
    /* (2) Configure MMS=010 to output a rising edge at each update event */
    /* (3) Select PCLK/960 i.e. 24MHz/960=25kHz */
    /* (4) Set one update event each second */
    /* (5) Enable TIM15 */
    RCC->APB2ENR |= RCC_APB2ENR_TIM15EN; /* (1) */ 
    TIM15->CR2 |= TIM_CR2_MMS_1; /* (2) */
    //TIM15->PSC = 959; /* (3) */  // The counter clock frequency is fCK_PSC/(PSC+1)
    //TIM15->ARR = (uint16_t)50000; /* (4) */
    TIM15->PSC = 47999;
    // TIM15->ARR = 1000; // every second
    TIM15->ARR = 20; // 50 times per second
    TIM15->CR1 |= TIM_CR1_CEN; /* (5) */

    gpio_set_PWR_DEN(1);
    /*** START THE PUMP ***/
    CurrentChannel = 0; /* Initializes the CurrentChannel */
    ChannelReadings = channel_readings;
    ADC1->CR |= ADC_CR_ADSTART; /* start the ADC conversions */
}


#include "spi_can.h"

void ADC1_COMP_IRQHandler(void)
{
    if ((ADC1->ISR & ADC_ISR_OVR) != 0)  /* checks OVR has triggered the IT */
    {
      ADC1->ISR |= ADC_ISR_EOC | ADC_ISR_EOSEQ | ADC_ISR_OVR; /* clears all pending flags */
      ADC1->CR |= ADC_CR_ADSTP; /* stop the sequence conversion */
      /* the data in the DR is considered as not valid */
      return;
    }
      
    if ((ADC1->ISR & ADC_ISR_EOC) != 0) /* checks EOC has triggered the IT */
    {
        ChannelReadings[CurrentChannel] = ADC1->DR; /* reads data and clears EOC flag */
        CurrentChannel++;  /* increments the index on ADC_array */        
    }

    if ((ADC1->ISR & ADC_ISR_EOSEQ) != 0)  /* checks EOSEQ has triggered the IT */
    {
        ADC1->ISR |= ADC_ISR_EOSEQ; /* clears the pending bit */
        CurrentChannel = 0; /* reinitialize the CurrentChannel */
        NOCAN->COUNTER++;
    }
}


