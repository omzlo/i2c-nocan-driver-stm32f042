#ifndef _GPIO_H_
#define _GPIO_H_

#define HIGH 1
#define LOW  0

void gpio_init(void);

void gpio_write_can_led(int val);

void gpio_write_can_int(int val);

void gpio_write_reset(int val);

#endif
