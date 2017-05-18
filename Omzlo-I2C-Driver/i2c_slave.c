#include <stm32f0xx.h>
//#include <stm32f0xx_rcc.h>
//#include <stm32f0xx_gpio.h>
//#include <stm32f0xx_misc.h>
#include "i2c_slave.h"
#include "nocan.h"
#include "systick.h"

uint16_t NOCAN_STATUS = 0;
#define NOCAN_STATUS_BAD_REQUEST (1<<0)

const i2c_op_callbacks *callback_array;
uint8_t                 callback_count;
uint8_t                 callback_op;
uint8_t                 callback_pos;

int i2c_slave_init(void)
{
    callback_count = 0;
    callback_op = 0;
    callback_pos = 0;

    /*---
    GPIO_InitTypeDef  GPIO_InitStructure;

    // Enable GPIO clock 
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF, ENABLE);

    // Connect I2C pins to AF1
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource0, GPIO_AF_1);
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource1, GPIO_AF_1); 

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //UP
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_Init(GPIOF, &GPIO_InitStructure);
    ---*/

    /* Enable the peripheral clock of GPIOB */
    RCC->AHBENR |= RCC_AHBENR_GPIOFEN;
    
    /* (1) open drain for I2C signals */
    /* (2) AF1 for I2C signals */
    /* (3) Select AF mode (10) on PB6 and PB7 */
    GPIOF->OTYPER |= GPIO_OTYPER_OT_0 | GPIO_OTYPER_OT_1; /* (1) */
    GPIOF->AFR[0] = (GPIOF->AFR[0] & ~(GPIO_AFRL_AFRL0 | GPIO_AFRL_AFRL1)) \
                  | (1 << ( 1 * 4 )) | (1 << (0 * 4)); /* (2) */
    GPIOF->MODER = (GPIOF->MODER & ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7)) \
                 | (GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1); /* (3) */



    /* Configure RCC for I2C1 */
    /* (1) Enable the peripheral clock I2C1 */
    /* (2) Use SysClk for I2C CLK */
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN; /* (1) */
    RCC->CFGR3 |= RCC_CFGR3_I2C1SW; /* (2) */

    /* Configure I2C1, slave */
    /* (2) Timing register value is computed with the AN4235 xls file,
           fast Mode @400kHz with I2CCLK = 48MHz, rise time = 140ns, fall time = 40ns */
    /* (3) Periph enable, receive interrupt enable */
    /* (4) 7-bit address = 0x12 */
    /* (5) Enable own address 1 */

    // * Initial value (400kHZ/48Mhz)
    // PRESC    0x0 
    // SCLDEL   0xB (11)
    // SDADEL   0x0
    // TIMINGR: 00B0 0000

    // * For 100kHz, with fI2CCLK=48Mhz
    // PRESC    0xB 
    // SCLDEL   0x4
    // SDADEL   0x2
    // TIMINGR: B042 0000

    // * For 400kHz, 48Mhz (from docs)
    // PRESC    0x5 
    // SCLDEL   0x3
    // SDADEL   0x3
    // TIMINGR: 5033 0000
    
    // GOOGLE SAYS [I2C_CLK_SRC_48MHZ] = {
    //  [I2C_FREQ_1000KHZ] = 0x50100103,
    //  [I2C_FREQ_400KHZ] = 0x50330309,
    //  [I2C_FREQ_100KHZ] = 0xB0421214

    I2C1->TIMINGR = (uint32_t)0xB0420000; /* (2) */
    I2C1->CR1 = I2C_CR1_RXIE | I2C_CR1_ADDRIE | I2C_CR1_STOPIE; /* (3) */
        // + I2C_CR1_PE -> Enable I2C
        // + I2C_CR1_RXIE -> Enable RECV interrupt enable
        // + I2C_CR1_ADDRIE -> Address match interrupt enable
        // + I2C_CR1_STOPIE -> Stop detection interrupt enable
    I2C1->CR1 |= I2C_CR1_PE;

    I2C1->OAR1 |= (uint32_t)(0x12<<1); /* (4) */
    I2C1->OAR1 |= I2C_OAR1_OA1EN; /* (5) */

    /* Configure IT */
    /* (7) Set priority for I2C1_IRQn */
    /* (8) Enable I2C1_IRQn */
    NVIC_SetPriority(I2C1_IRQn, 0); /* (7) */
    NVIC_EnableIRQ(I2C1_IRQn); /* (8) */
}


void i2c_slave_reset(void)
{
    I2C1->CR1 &= ~I2C_CR1_PE;
    callback_op = 0;
    systick_delay(100);
    I2C1->CR1 |= I2C_CR1_PE;
}

void i2c_slave_set_callbacks(uint8_t cb_count, const i2c_op_callbacks *cb_array)
{
    callback_array = cb_array;
    callback_count = cb_count;
    callback_op = 0;
}

void I2C1_IRQHandler(void)
{
    // See page 669 in RM0091 reference manual for interrup clear/set conditions

    uint32_t I2C_InterruptStatus = I2C1->ISR; /* Get interrupt status */
    volatile uint32_t data;

    if ((I2C_InterruptStatus & I2C_ISR_ADDR) == I2C_ISR_ADDR)
    {
        I2C1->ICR |= I2C_ICR_ADDRCF; /* Address match event */
        callback_pos = 0;

        if((I2C1->ISR & I2C_ISR_DIR) == I2C_ISR_DIR) /* Check if transfer direction is read (slave transmitter) */
        {
            I2C1->ISR |= I2C_ISR_TXE;  /* flush any data in TXDR */
            I2C1->CR1 |= I2C_CR1_TXIE; /* Set transmit IT */
        }

        NOCAN->I2C_START_COUNT++;
    }
    else if ((I2C_InterruptStatus & I2C_ISR_RXNE) == I2C_ISR_RXNE)
    {
        /* Read receive register, will clear RXNE flag */
        data = (I2C1->RXDR)&0xFF;
        if (callback_pos==0)
        {
            if (data<callback_count)
                callback_op = data;
        }
        callback_array[callback_op].recv_cb(callback_pos++, data);
    }
    else if ((I2C_InterruptStatus & I2C_ISR_TXIS) == I2C_ISR_TXIS)
    {
        data = callback_array[callback_op].send_cb(callback_pos++);
        I2C1->TXDR = data & 0xff; /* Byte to send */
    }
    else if ((I2C_InterruptStatus & I2C_ISR_STOPF) == I2C_ISR_STOPF)
    {
         I2C1->CR1 &=~ I2C_CR1_TXIE; /* Disable transmit IT */
         // write STOPCF = 1 to clear interrupt
         I2C1->ICR |= I2C_ICR_STOPCF;
         callback_array[callback_op].stop_cb(callback_pos);
         callback_op = 0;
        
         NOCAN->I2C_STOP_COUNT++;
    }
    /*else
    {
        i2c_slave_reset(); // also set callback_op to 0
    }*/
}

