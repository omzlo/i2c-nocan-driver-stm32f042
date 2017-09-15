#include "power.h"
#include "gpio.h"
#include "registers.h"
#include <stm32f0xx.h>

void power_on(void)
{
    REGS.STATUS[0] &= ~(1<<STATUS0_POWER_FAULT);
    REGS.STATUS[1] &= ~(1<<STATUS1_LED_RED);
    REGS.STATUS[1] |= ((1<<STATUS1_LED_YELLOW) | (1<<STATUS1_POWER));

    gpio_set_LED_YEL(1);
    gpio_set_LED_RED(0);
    gpio_set_PWR_DEN(1);
    gpio_set_PWR_IN(1);
    
    ADC1->CFGR1 |= ADC_CFGR1_AWDEN;     // enable analog watchdog
}

void power_off(void)
{
    ADC1->CFGR1 &= ~ADC_CFGR1_AWDEN;    // disable analog watchdog

    gpio_set_PWR_IN(0);
    gpio_set_LED_YEL(0);
    REGS.STATUS[1] &= ~((1<<STATUS1_LED_YELLOW) | (1<<STATUS1_POWER));
}

void power_fault(void)
{
    power_off();
    REGS.STATUS[0] |= (1<<STATUS0_POWER_FAULT);
    REGS.STATUS[1] |= (1<<STATUS1_LED_RED);
    gpio_set_LED_RED(1);
}

