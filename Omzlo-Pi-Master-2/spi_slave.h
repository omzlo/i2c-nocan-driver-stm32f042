#ifndef _SPI_SLAVE_H_
#define _SPI_SLAVE_H_
#include <stdint.h>

int spi_slave_init(void);

#define SPI_OP_TEST         0x0
#define SPI_OP_SEND         0x1
#define SPI_OP_ENABLE       0x2
#define SPI_OP_DISABLE      0x3
#define SPI_OP_RECV         0x4
//#define SPI_OP_NOP5         0x5
//#define SPI_OP_NOP6         0x6
#define SPI_OP_RESET        0x7
#define SPI_OP_READ         0x8

/*
   SEND_PACKET, A5
   [0 0 0 1 | 0 0 0 0] P0, P1, ... P16

   ENABLE_FUNC, b4
   [0 0 1 0][0-15]

   DISABLE_FUNC, b4
   [0 0 1 1][0-15]

   RECV_PACKET
   [0 1 0 0 | 0 0 0 0] P0, P1, ...

   RESET
   [0 1 1 1 | 1 1 1 1]

   READ_REGS, A7          // 127
   [1][x x x x x x x], ...
*/

#endif
