#ifndef _SPI_CAN_H_
#define _SPI_CAN_H_

#include <stdint.h>

#define INIT_PHASE_DATA 0
#define INIT_PHASE_SPI  1
#define INIT_PHASE_CAN  2

void spi_can_init(int phase);

typedef struct __attribute__((packed)) {
    uint8_t SIGNATURE[4];       // 0
    uint8_t VERSION_MAJOR;      // 4
    uint8_t VERSION_MINOR;      // 5
    uint8_t CHIP_UDID[12];      // 6
    volatile uint8_t STATUS;    // 18, status register
    uint8_t RESERVED;           // 19
    uint16_t LEVELS[3];         // 20
    uint16_t GUARD;             // 26
    uint32_t COUNTER;           // 28
                                // 32
} nocan_registers_t;

extern nocan_registers_t *NOCAN;

#endif
