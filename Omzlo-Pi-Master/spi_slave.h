#ifndef _SPI_SLAVE_H_
#define _SPI_SLAVE_H_
#include <stdint.h>

typedef uint8_t (*spi_transfer_cb)(uint8_t, uint8_t);  // (offset,r_value) -> w_value

int spi_slave_init(void);

void spi_slave_set_callbacks(uint8_t cb_count, const spi_transfer_cb *callbacks);

void spi_slave_configure_EXTI(void);

#endif
