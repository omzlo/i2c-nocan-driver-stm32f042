#include "registers.h"
#include "gpio.h"
#include "power.h"

registers_typdef REGS;
uint8_t *REGS_BYTE = (uint8_t *)&REGS;

#define REGS_SIZE sizeof(REGS)


extern inline uint8_t registers_read(uint16_t addr);

extern inline void registers_write(uint16_t addr, uint8_t value);

void registers_function_enable(uint8_t fn)
{
    switch (fn) {
        case STATUS1_LED_YELLOW:
            REGS.STATUS[1] |= (1<<STATUS1_LED_YELLOW);
            gpio_set_LED_YEL(1);
            break;

        case STATUS1_LED_RED:
            REGS.STATUS[1] |= (1<<STATUS1_LED_RED);
            gpio_set_LED_RED(1);
            break;

        case STATUS1_RES_OFF:
            REGS.STATUS[1] |= (1<<STATUS1_RES_OFF);
            gpio_set_CAN_RES(1);
            break;

        case STATUS1_POWER:
            power_on();
            break;
    }
}

void registers_function_disable(uint8_t fn)
{
    switch (fn) {
        case STATUS1_LED_YELLOW:
            REGS.STATUS[1] &= ~(1<<STATUS1_LED_YELLOW);
            gpio_set_LED_YEL(0);
            break;

        case STATUS1_LED_RED:
            REGS.STATUS[1] &= ~(1<<STATUS1_LED_RED);
            gpio_set_LED_RED(0);
            break;

        case STATUS1_RES_OFF:
            REGS.STATUS[1] &= ~(1<<STATUS1_RES_OFF);
            gpio_set_CAN_RES(0);
            break;

        case STATUS1_POWER:
            power_off();
            break; 
    }
    
}

/****/

can_packet_typedef *registers_recv_back(void)
{
    can_packet_typedef *retval = 0;

    // are we not full ?
    if (((REGS.RECV_TAIL+1)&(RECV_BUFFER_SIZE-1)) != REGS.RECV_HEAD)
    {
        retval = &REGS.RECV_BUFFER[REGS.RECV_TAIL];
    }
    return retval;
}

void registers_recv_push_back(void)
{
    // are we not full ?
    if (((REGS.RECV_TAIL+1)&(RECV_BUFFER_SIZE-1)) != REGS.RECV_HEAD)
    {
        REGS.RECV_TAIL = (REGS.RECV_TAIL+1) & (RECV_BUFFER_SIZE-1);
    }
    else
    {
        REGS.STATUS[0] |= STATUS0_RECV_FULL;
    }
    REGS.STATUS[0] |= STATUS0_RECV_PENDING;
    gpio_set_CAN_RX_INT(LOW);
}

void registers_recv_pop_front(void)
{
    // are we not empty ?
    if (REGS.RECV_TAIL != REGS.RECV_HEAD)
    {
        REGS.RECV_HEAD = (REGS.RECV_HEAD+1) & (RECV_BUFFER_SIZE-1);
        if (REGS.RECV_TAIL==REGS.RECV_HEAD)
        {
            REGS.STATUS[0] &= ~STATUS0_RECV_PENDING;
            gpio_set_CAN_RX_INT(HIGH);
        }
        REGS.STATUS[0] &= ~STATUS0_RECV_FULL;
    }
}

uint16_t registers_recv_front(void)
{
    uint16_t retval = BAD_ADDRESS;

    // are we not empty
    if (REGS.RECV_TAIL != REGS.RECV_HEAD)
    {
        retval = REGS_OFFSET(REGS.RECV_BUFFER[REGS.RECV_HEAD]);
    }
    return retval;
}

/****

uint16_t registers_send_back(void)
{
    uint16_t retval = BAD_ADDRESS;

    // are we not empty
    if (((REGS.SEND_TAIL+1)&(SEND_BUFFER_SIZE-1)) != REGS.SEND_HEAD)
    {
        retval = REGS_OFFSET(REGS.SEND_BUFFER[REGS.SEND_TAIL]);
    }
    return retval;
}

void registers_send_push_back(void)
{
    if (((REGS.SEND_TAIL+1)&(SEND_BUFFER_SIZE-1)) != REGS.SEND_HEAD)
    {
        REGS.SEND_TAIL = (REGS.SEND_TAIL+1) & (SEND_BUFFER_SIZE-1);
        REGS.STATUS[0] |= STATUS0_SEND_PENDING;
    }
    else
    {
        REGS.STATUS[0] |= STATUS0_SEND_FULL;
        gpio_set_CAN_TX_INT(1);
    }
}


can_packet_typedef *registers_send_front(void)
{
    if (REGS.SEND_TAIL != REGS.SEND_HEAD)
    {
        return REGS.SEND_BUFFER + REGS.SEND_HEAD;
    }
    return 0;
}


void registers_send_pop_front(void)
{
    if (REGS.SEND_TAIL != REGS.SEND_HEAD)
    {
        REGS.SEND_HEAD = (REGS.SEND_HEAD+1) & (RECV_BUFFER_SIZE-1);
        if (REGS.SEND_TAIL == REGS.SEND_HEAD)
        {
            REGS.STATUS[0] &= ~STATUS0_SEND_PENDING;
        }
        REGS.STATUS[0] &= ~STATUS0_SEND_FULL;
        gpio_set_CAN_TX_INT(0);
    }
}

***/

uint16_t registers_send_prepare(void)
{
    if ((REGS.STATUS[0] & STATUS0_SEND_PENDING)==0)
    {
        return REGS_OFFSET(REGS.SEND_BUFFER);
    }
    return BAD_ADDRESS;
}

void registers_send_execute(void)
{
    REGS.STATUS[0] |= STATUS0_SEND_PENDING;
    gpio_set_CAN_TX_INT(LOW);
    can_send(&REGS.SEND_BUFFER);
}

void registers_send_done(void)
{
    REGS.STATUS[0] &= ~STATUS0_SEND_PENDING;
    gpio_set_CAN_TX_INT(HIGH);
}

/***/

const uint8_t *CHIP_UDID = (const uint8_t *)0x1FFFF7AC;

void registers_init(void)
{
    int i;

    _Static_assert(sizeof(REGS)==283, "Registers are expected to be 283 bytes in size.");

    for (i=0;i<REGS_SIZE;i++)
        REGS_BYTE[i]=0;

    REGS.SIGNATURE[0] = 'C';
    REGS.SIGNATURE[1] = 'A';
    REGS.SIGNATURE[2] = 'N';
    REGS.SIGNATURE[3] = '0';
    REGS.VERSION = 0x1;

    for (i=0;i<12;i++)
        REGS.CHIP_UDID[i] = CHIP_UDID[i];

    REGS.LEVELS[3] = *((uint16_t *)(0x1FFFF7BA));   // VREFINT_CAL for STM32F072
    REGS.LEVELS[4] = 3096;

    REGS.MARKER = 0xEFBEADDE;
}

