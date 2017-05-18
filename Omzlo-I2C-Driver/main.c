#include "can.h"
#include "i2c_slave.h"
#include "chip_options.h"
#include "systick.h"
#include "gpio.h"
#include "nocan.h"
#include "usart.h"


int main(void)
{
  int n = 0;
  char c;

  systick_init();
  
  usart_init(115200);
  
  chip_options_configure();

  gpio_init();

  i2c_slave_init();

  can_init();

  nocan_init();
  
  //i2c_slave_set_callbacks(1, op_table);

  usart_printf("Starting serial console.\n");
  for(;;)
  {
      usart_printf("\n[h,c,d,i,q,r,z]?");
      c = usart_getc();
      switch (c) {
          case 'd': 
              usart_printf("\nregisters: ");
              for (int i=0;i<sizeof(*NOCAN);i++)
                  usart_printf("%x ", ((uint8_t *)NOCAN)[i]);
              break;
          case 'z':
              usart_printf("\nreseting self (stm32f0)...");
              NVIC_SystemReset();
              break;
          case 'h': 
              usart_printf("\ncommands: [c] send address request [h] help, [i] read chip uid, [d] dump registers, [p] 1 second systick pause [q] i2c slave reset [r] reset atmega328pb [z] reset stm32f0."); 
              break;
          case 'c':
              usart_printf("\nconnecting...");
              nocan_sys_send(NOCAN->NODE_ID, NOCAN_SYS_ADDRESS_REQUEST, 0, NOCAN->UDID, 8);
              break;
          case 'p':
              usart_printf("\npause: ");
              systick_delay(1000);
              usart_printf("OK");
              break;
          case 'q':
              usart_printf("\nreset i2c: ");
              i2c_slave_reset();
              break;
          case 'r':
              gpio_write_can_led(LOW);
              usart_printf("\nreseting atmega328pb");
              gpio_write_reset(LOW);
              for (uint32_t count=0;count<10000;count++) asm("nop");
              gpio_write_reset(HIGH);
              break;
         case 'i':
              usart_printf("\nChip UID: ");
              for (int i=0;i<12;i++) 
                  usart_printf("%x ", CHIP_UDID[i]);
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

