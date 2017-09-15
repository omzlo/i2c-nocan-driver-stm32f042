#ifndef _GPIO_H_
#define _GPIO_H_

#define HIGH 1
#define LOW  0

void gpio_init(void);

void gpio_set_LED_YEL(int val);

void gpio_set_LED_RED(int val);

void gpio_set_CAN_RX_INT(int val);

void gpio_set_CAN_TX_INT(int val);

void gpio_set_PWR_IN(int val);

void gpio_set_PWR_DEN(int val);

#endif
