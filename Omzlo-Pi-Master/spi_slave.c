#include <stm32f0xx.h>
#include "spi_slave.h"
#include "usart.h"

// typedef uint8_t (*spi_transfer_cb)(uint8_t, uint8_t);  // (offset,r_value) -> w_value
// SPI_SS on PA4
// SPI1_SCK on PA5
// SPI1_MISO on PA6
// SPI1_MOSI on PA7


const spi_transfer_cb *callback_array = 0;
uint8_t callback_pos = 0;
uint8_t callback_count = 0;
uint8_t callback_op = 0;


void spi_slave_configure_EXTI(void)
{
  /* (1) PA4 as source input */
  /* (2) unmask port 0 */
  /* (3) Rising edge */
  /* (4) Set priority */
  /* (5) Enable EXTI0_1_IRQn */

  SYSCFG->EXTICR[1] = (SYSCFG->EXTICR[1] & ~SYSCFG_EXTICR2_EXTI4) | SYSCFG_EXTICR2_EXTI4_PA; /* (1) */ 
  //  SYSCFG->EXTICR[0] => PA0, PB0, PC0, ... PF0 as specified in bits [0:3] of  SYSCFG_EXTICR1 
  //                    => PA1, PB1, PC1, ... PF1 as specified in bits [4:7]
  //                    ...
  //                    => PA3, PB3, PC3, ... PF3 as specified in bits [12:15]
  //
  //  SYSCFG->EXTICR[1] => PA4, PB4, PC4, ... PF4 as specified in bits [0:3] of  SYSCFG_EXTICR2 
  //  
    
  
  
  EXTI->IMR |= EXTI_IMR_MR4; /* (2) */
  // Interrupt request form line _0_ is unmasked (1)
  // SYSCFG->EXTICR selects the letter PA, PB, ... PF and here we select one or more pins 
  // for the letter  (incorrect)

  EXTI->RTSR |= EXTI_RTSR_TR4; /* (3) */ 
  // Rising edge on line _0_
  // EXTI->FTSR for falling edge
  EXTI->FTSR |= EXTI_FTSR_TR4;

  NVIC_SetPriority(EXTI4_15_IRQn, 0); /* (4) */ 
  // EXTI0_1 covers interrupts on pins Px0 and Px1
  // EXTI2_3 covers interrupts on pins Px2 and Px3
  // EXTI4_15 coverts interrupts on pins Px4, Px5, Px6, ..., Px15
  // Priority 0 is the highest (as here), priority 3 is the lowest 
  //=// NVIC_EnableIRQ(EXTI0_1_IRQn); /* (5) */ 
  NVIC_EnableIRQ(EXTI4_15_IRQn); /* (5) */ 
}

void EXTI4_15_IRQHandler(void)
{
    EXTI->PR |= EXTI_PR_PR4;
    if ((GPIOA->IDR & GPIO_IDR_4)==0)
    {
        SPI1->CR1 &= ((uint16_t)0xFEFF);
        callback_pos = 0;
    }
    else
    {   
        SPI1->CR1 |= SPI_CR1_SSI;
    }
}

int spi_slave_init()
{

    /* Enable the peripheral clock of GPIOA */
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    /* Select AF mode (10) on PA4, PA5, PA6, PA7 */
    GPIOA->MODER = (GPIOA->MODER 
            & ~(GPIO_MODER_MODER5 | GPIO_MODER_MODER6 | GPIO_MODER_MODER7))
        | (GPIO_MODER_MODER5_1| GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1);

    /* AF0 for SPI1 signals */
    GPIOA->AFR[1] = (GPIOA->AFR[1] &
            ~(GPIO_AFRL_AFRL5 | GPIO_AFRL_AFRL6 | GPIO_AFRL_AFRL7)); 


    /* Enable input for GPIO PA4 */
    // Nothing to do since default state
    GPIOA->MODER &=  ~(GPIO_MODER_MODER4); 


    /* Enable the peripheral clock SPI1 */
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    /* Configure SPI1 in slave */
    /* nSS hard, slave, CPOL and CPHA at zero (rising first edge) */
    /* (1) RXNE IT, 8-bit Rx fifo */
    /* (2) Enable SPI1 */
    SPI1->CR2 = SPI_CR2_RXNEIE                          // Enable RX buffer not empty interrupt
        | SPI_CR2_FRXTH                                 // RXNE event generated if FIFO level = 8 bits
        | SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0    // DataSize=8 bits
        ; /* (1) */
    SPI1->CR1 |= (SPI_CR1_SPE                            // SPI enable
        | SPI_CR1_SSM)                                  // Software Slave Select
        ; /* (2) */

    /* Configure IT */
    /* (3) Set priority for SPI1_IRQn */
    /* (4) Enable SPI1_IRQn */
    NVIC_SetPriority(SPI1_IRQn, 0); /* (3) */
    NVIC_EnableIRQ(SPI1_IRQn); /* (4) */

    spi_slave_configure_EXTI();
    return 0;
}

void spi_slave_set_callbacks(uint8_t cb_count, const spi_transfer_cb *callbacks)
{
    callback_array = callbacks;
    callback_count = cb_count;
    callback_op = 0;
    callback_pos = 0;
}

/**
 *   * @brief  This function handles SPI1 interrupt request.
 *   * @param  None
 *   * @retval None
 *   */

void SPI1_IRQHandler(void)
{
    uint8_t SPI1_Data = 0;

    if((SPI1->SR & SPI_SR_RXNE) == SPI_SR_RXNE)
    {
        SPI1_Data = (uint8_t)SPI1->DR; /* Receive data, clear flag */

        //*(uint8_t *)&(SPI1->DR) = SPI1_Data^0xFF;
        if (callback_pos==0)
        {
            callback_op = SPI1_Data;
        }
        if (callback_op<callback_count)
            *(uint8_t *)&(SPI1->DR) = callback_array[callback_op](callback_pos++, SPI1_Data);
        else
            *(uint8_t *)&(SPI1->DR) = 0xEE;
    }
}


