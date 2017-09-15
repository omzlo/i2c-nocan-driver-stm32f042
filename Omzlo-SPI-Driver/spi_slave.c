#include <stm32f0xx.h>
#include "spi_slave.h"
#include "usart.h"

extern inline void spi_send_dma(uint8_t len, void *data);
extern inline void spi_send_byte(uint8_t byte);
extern inline uint8_t spi_recv_byte(void);

// typedef uint8_t (*spi_transfer_cb)(uint8_t, uint8_t);  // (offset,r_value) -> w_value
// SPI_SS on PA4
// SPI1_SCK on PA5
// SPI1_MISO on PA6
// SPI1_MOSI on PA7

const spi_callbacks_typedef *callback_array = 0;
uint8_t callback_count = 0;
volatile uint8_t callback_op = 0;


static void spi_slave_configure_DMA(void)
{
    /* Enable the peripheral clock DMA1 */
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;


    /* DMA1 Channel2 SPI1_RX config */
    /* (1) Peripheral address */
    /* (2) Memory address */
    /* (3) Data size */
    /* (4) Memory increment */
    /*     Peripheral to memory */
    /*     8-bit transfer */
    /*     Transfer complete IT */
    DMA1_Channel2->CPAR = (uint32_t)&(SPI1->DR); /* (1) */
    DMA1_Channel2->CMAR = 0; /* (2) */
    DMA1_Channel2->CCR |= DMA_CCR_MINC |  DMA_CCR_TCIE; /* (4) */


    /* DMA1 Channel3 SPI1_TX config */
    /* (5) Peripheral address */
    /* (7) Memory increment */
    /*     Memory to peripheral*/
    /*     8-bit transfer */    
    DMA1_Channel3->CPAR = (uint32_t)&(SPI1->DR); /* (5) */
    DMA1_Channel3->CMAR = 0;
    DMA1_Channel3->CCR |= DMA_CCR_MINC | DMA_CCR_DIR; /* (7) */

    /* Configure IT */
    /* (8) Set priority for DMA1_Channel2_3_IRQn */
    /* (9) Enable DMA1_Channel2_3_IRQn */
    NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0); /* (8) */
    NVIC_EnableIRQ(DMA1_Channel2_3_IRQn); /* (9) */
}


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

  // Rising edge on line _0_
  // EXTI->FTSR for falling edge
  EXTI->RTSR |= EXTI_RTSR_TR4; /* (3) */ 
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
    uint8_t dummy;

    EXTI->PR |= EXTI_PR_PR4;
    if ((GPIOA->IDR & GPIO_IDR_4)==0)
    {
        SPI1->CR1 &= ((uint16_t)0xFEFF);        // Software SS clear

        while ((SPI1->SR & SPI_SR_RXNE) == 0);
        callback_op = (uint8_t)SPI1->DR; /* Receive data, clear flag */

        if (callback_op < callback_count)
            callback_array[callback_op].transfer_cb(callback_op);
        else
            *(uint8_t *)&(SPI1->DR) = 0xEE;
    }
    else
    {   
        SPI1->CR1 |= SPI_CR1_SSI;
        DMA1_Channel3->CCR &=~ DMA_CCR_EN; // Disable TX DMA
        while ((SPI1->SR & SPI_SR_RXNE) != 0) dummy = (uint8_t)SPI1->DR;
    }
}

void DMA1_Channel2_3_IRQHandler(void)
{
  if((DMA1->ISR & DMA_ISR_TCIF2) == DMA_ISR_TCIF2)
  {
    DMA1->IFCR |= DMA_IFCR_CTCIF2; /* Clear TC flag */

    DMA1_Channel2->CCR &=~ DMA_CCR_EN;  // disable RX DMA
    if (callback_op < callback_count)
        callback_array[callback_op].finalize_cb();
  }
}


int spi_slave_init()
{

    /* Enable the peripheral clock of GPIOA */
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    /* Select AF mode (10) on PA5, PA6, PA7 */
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
        | SPI_CR2_TXDMAEN                               // DMA on TX
        ; /* (1) */
    SPI1->CR1 |= (SPI_CR1_SPE                            // SPI enable
        | SPI_CR1_SSM)                                  // Software Slave Select
        ; /* (2) */

    /* Configure IT */
    /* (3) Set priority for SPI1_IRQn */
    /* (4) Enable SPI1_IRQn */
    //NVIC_SetPriority(SPI1_IRQn, 0); /* (3) */
    //NVIC_EnableIRQ(SPI1_IRQn); /* (4) */

    spi_slave_configure_DMA();
    spi_slave_configure_EXTI();
    return 0;
}

void spi_slave_set_callbacks(uint8_t cb_count, const spi_callbacks_typedef *callbacks)
{
    callback_array = callbacks;
    callback_count = cb_count;
    callback_op = 0;
}

/**
 *   * @brief  This function handles SPI1 interrupt request.
 *   * @param  None
 *   * @retval None
 *   */
#if 0
void SPI1_IRQHandler(void)
{
    uint8_t SPI1_Data = 0;

    if((SPI1->SR & SPI_SR_RXNE) == SPI_SR_RXNE)
    {
        SPI1_Data = (uint8_t)SPI1->DR; /* Receive data, clear flag */

        
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
#endif


