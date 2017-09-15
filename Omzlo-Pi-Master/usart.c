#include <stm32f0xx.h>
#include <stm32f0xx_rcc.h>
#include <stm32f0xx_gpio.h>
#include <stm32f0xx_usart.h>
#include "usart.h"

unsigned usart_debug_enable = 0;

/* NOTES:
 * - Configures PA0(TX) and PA1(RX) as USART4
 */

int usart_init(uint32_t baud)
{
    USART_InitTypeDef USART_InitStructure; 
    GPIO_InitTypeDef GPIO_InitStructureTx; 
    GPIO_InitTypeDef GPIO_InitStructureRx;
    

    /* Enable the peripheral clock of GPIOA */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART4, ENABLE); 

    /* USART 4 is on Alternate Function 4 */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_4);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_4);

    // Configure TX pin

    GPIO_InitStructureTx.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructureTx.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructureTx.GPIO_Speed = GPIO_Speed_50MHz; 
    GPIO_InitStructureTx.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructureTx.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructureTx);

    // Configure RX pin

    GPIO_InitStructureRx.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructureRx.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructureRx.GPIO_Speed = GPIO_Speed_50MHz; 
    GPIO_InitStructureRx.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructureRx.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructureRx);

    // Configure the UART

    USART_StructInit(&USART_InitStructure); 
    USART_InitStructure.USART_BaudRate = baud;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode  = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART4,&USART_InitStructure); 
    USART_Cmd(USART4, ENABLE); 

    return 0;    
}

int usart_getc (void)
{
    while (USART_GetFlagStatus(USART4, USART_FLAG_RXNE) == RESET);
    return  USART4->RDR & 0xff;
}

int usart_available(void)
{
    return USART_GetFlagStatus(USART4, USART_FLAG_RXNE) == SET;
}

int usart_putc(int c)
{
  while (USART_GetFlagStatus(USART4, USART_FLAG_TXE) == RESET);
  USART_SendData(USART4, c);
  return 0;
}

#ifndef NULL
#define NULL ((void *)0)
#endif

const char digits[]="0123456789abcdef";

static void _process_uint(unsigned u)
{
    unsigned d,c;

    if (u==0) {
        usart_putc('0');
        return;
    }

    d=1000000000; // 32 bits means max uint = 4,294,967,295
    while (d>u) d/=10;
    while (d)
    {
        c = u/d;
        usart_putc(digits[c]);
        u %= d;
        d /= 10;
    }
    return;
}

static void _process_string(const char *s)
{
    while (*s) usart_putc(*s++);
}

static void _process_hex(unsigned u)
{
    unsigned su,c;

    if (u==0) 
    {
        usart_putc('0');
        usart_putc('0');
        return;
    }         
    su = 24;
    while ((u>>su)==0) su-=8;
    su+=4;
    for (;;) {
        c = (u>>su)&0xF;
        usart_putc(digits[c]);
        if (su==0) break;
        su-=4;
    }
}

int usart_vprintf(const char *format, va_list ap)
{   
    int i;
    unsigned u;
    const char *s;

    while (*format) {
        if (*format=='%')
        {
            format++;
            switch (*format) {
                case '%':
                    format++;
                    usart_putc(*format++);
                    break;
                case 'i':
                    format++;
                    i = va_arg(ap, int);
                    if (i<0)
                    {
                        usart_putc('-');
                        i = -i;
                    }
                    _process_uint((unsigned)i);
                    break;
                case 'u':
                    format++;
                    u = va_arg(ap, unsigned);
                    _process_uint(u);
                    break;
                case 'x':
                case 'X':
                    format++;
                    u = va_arg(ap, unsigned);
                    _process_hex(u);
                    break;
                case 's':
                    format++;
                    s = va_arg(ap, const char *);
                    _process_string(s);    
                    break;
                case 'c':
                    format++;
                    i = va_arg(ap, int);
                    usart_putc(i);
                    break;
                case 'p':
                    format++;
                    _process_string("0x");
                    u = va_arg(ap, unsigned);
                    _process_hex(u);
                    break;
                default:
                    format++;
                    _process_string("<?format?>");
                    return -1;
            }
        }
        else
        {
            if (*format=='\n')
                usart_putc('\r');
            usart_putc(*format++);
        }
    }

    return 0;
}


int usart_printf(const char *format, ...)
{
    va_list ap;
    int retval;

    va_start(ap, format);
    retval = usart_vprintf(format, ap);
    va_end(ap);
    return retval;
}

int usart_debug_printf(const char *format, ...)
{
    if (usart_debug_enable)
    {
        va_list ap;
        int retval;

        va_start(ap, format);
        retval = usart_vprintf(format, ap);
        va_end(ap);
        return retval;
    }
    return 0;
}


