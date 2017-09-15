#ifndef _REGISTERS_H_
#define _REGISTERS_H_

#include <stdint.h>
#include "can.h"

#define RECV_BUFFER_SIZE 16

#define BAD_ADDRESS 0xFFFF

#define STATUS0_RECV_PENDING 1
#define STATUS0_SEND_PENDING 2
#define STATUS0_RECV_FULL    3
#define STATUS0_ADC_FAULT    6
#define STATUS0_POWER_FAULT  7

#define STATUS1_LED_YELLOW   1
#define STATUS1_LED_RED      2
#define STATUS1_RES_OFF      3
#define STATUS1_POWER        7

#define STATUS2_SEND_ENQUEUE 1
#define STATUS2_RECV_SHIFT   2

typedef struct __attribute__((packed)) {
    // 0:
    uint8_t SIGNATURE[4];
    // +4 = 4:
    uint32_t VERSION;
    // +4 = 8:
    uint8_t CHIP_UDID[12];
    // + 12 = 20:
    volatile uint8_t STATUS[4];
    // +4 = 24:
    uint16_t LEVELS[6]; // Voltage(0), Current(1), VRef(2),  VRefCal(3), Watchdog limit(4), FaultLevel(5).
    // +12 = 36: 
    uint32_t DEBUG[4];
    // +16 = 52:
    uint8_t RECV_SIZE;
    uint8_t SEND_SIZE;
    uint8_t FILTER_SIZE;
    uint8_t RESERVED;
    // +4 = 56:
    volatile uint8_t RECV_HEAD;
    volatile uint8_t RECV_TAIL;
    // +2 = 58:
    can_packet_typedef RECV_BUFFER[RECV_BUFFER_SIZE];
    // +16*13 = 266:
    can_packet_typedef SEND_BUFFER;
    // +13 =279:
    uint32_t MARKER;
    // +4 = 283:
} registers_typdef;



extern registers_typdef REGS;

extern uint8_t *REGS_BYTE;

#define REGS_OFFSET(x) (uint16_t)((uint8_t *)(&(x))-REGS_BYTE)


void registers_init(void);

/***/

can_packet_typedef *registers_recv_back(void);

void registers_recv_push_back(void);

void registers_recv_pop_front(void);

uint16_t registers_recv_front(void);

/***

uint16_t registers_send_back(void);

void registers_send_push_back(void);

void registers_send_pop_front(void);

can_packet_typedef *registers_send_front(void);

***/

uint16_t registers_send_prepare(void);

void registers_send_execute(void);

void registers_send_done(void);

/***/

inline uint8_t registers_read(uint16_t addr) __attribute__((always_inline));
inline uint8_t registers_read(uint16_t addr)
{
    if (addr<sizeof(REGS))
        return REGS_BYTE[addr];
    else
        return 0xFF;
}

inline void registers_write(uint16_t addr, uint8_t value) __attribute__((always_inline));
inline void registers_write(uint16_t addr, uint8_t value)
{
    if (addr>=REGS_OFFSET(REGS.SEND_BUFFER) && addr<sizeof(REGS))
        REGS_BYTE[addr]=value;
}

void registers_function_enable(uint8_t fn);

void registers_function_disable(uint8_t fn);

#endif
