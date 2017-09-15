#include <stm32f0xx.h>
#include <stm32f0xx_rcc.h>
#include <stm32f0xx_gpio.h>
#include "gpio.h"
#include "systick.h"
#include "usart.h"
#include "spi_slave.h"
#include "spi_can.h"
#include "adc.h"
#include "power.h"

int main(void)
{
  char c;
  int i;

  systick_init();


  usart_init(115200);  
  usart_printf("\nStarting:\n");

  usart_printf("* Init GPIO: ");
  gpio_init();
  usart_printf("[OK]\n");

  usart_printf("* 3 second pause: ");
  for (i=0;i<3;i++)
  {
    gpio_set_LED_RED(1);
    gpio_set_LED_YEL(0);
    systick_delay(500);
    gpio_set_LED_RED(0);
    gpio_set_LED_YEL(1);
    systick_delay(500);
    usart_putc('.');
  }
  gpio_set_LED_YEL(0);
  usart_printf(" [OK]\n");
  
  usart_printf("* Init SPI slave: ");
  spi_slave_init();
  usart_printf("[OK]\n");

  usart_printf("* Init driver registers: ");
  spi_can_init(INIT_PHASE_DATA); 
  usart_printf("[OK]\n");

  usart_printf("* Init SPI callbacks: ");
  spi_can_init(INIT_PHASE_SPI); 
  usart_printf("[OK]\n");

  usart_printf("* Init CAN bus: ");
  spi_can_init(INIT_PHASE_CAN); 
  usart_printf("[OK]\n");


 
  usart_printf("* Init ADC: ");
  adc_init(NOCAN->LEVELS);
  usart_printf("[OK]\n");

  usart_printf("DONE\n");
  while(1)
  {
      usart_printf("\n[d,h,l,p,P,z]?");
     
      while (!usart_available())
      {
        power_evaluate();
      }
      
      c = usart_getc();
      switch (c) {
          case 'd':
              usart_printf("\nregisters: ");
              for (int i=0;i<sizeof(*NOCAN);i++)
                  usart_printf("%x ", ((uint8_t *)NOCAN)[i]);
              break;
          case 'h': 
              usart_printf("\ncommands: [h] help, [d] dump registers, [l] blink led [p/P] switch power off/on [r/R] disable/enable can-bus resistor [z] reset stm32f0."); 
              break;
          case 'l':
              usart_printf("\nblink red led 2 times.");
              gpio_set_LED_RED(1);
              systick_delay(1000);
              gpio_set_LED_RED(0);
              systick_delay(1000);
              gpio_set_LED_RED(1);
              systick_delay(1000);
              gpio_set_LED_RED(0);
              break;
          case 'p':
              usart_printf("\npower off.");
              power_off();
              break;
          case 'P':
              usart_printf("\npower on.");
              power_on();
              break; 
          case 'z':
              usart_printf("\nreseting self (stm32f0)...");
              NVIC_SystemReset();
              break;
      }
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

