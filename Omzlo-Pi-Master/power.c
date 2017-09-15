#include "power.h"
#include "gpio.h"
#include "spi_can.h"

void power_on(void)
{
    NOCAN->RESERVED = 0;
    gpio_set_LED_YEL(1);
    gpio_set_LED_RED(0);
    gpio_set_PWR_DEN(1);
    gpio_set_PWR_IN(1);
}

void power_off(void)
{
    gpio_set_PWR_IN(0);
    gpio_set_LED_YEL(0);
}

void power_evaluate(void)
{
    if (NOCAN->LEVELS[1]>4096-512)
    {
        power_off();
        NOCAN->RESERVED = 1;
        gpio_set_LED_RED(1);
    }
}

