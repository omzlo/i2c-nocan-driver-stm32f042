#ifndef _I2C_SLAVE_H_
#define _I2C_SLAVE_H_
#include <stdint.h>

#define I2C_DIR_WRITE 0
#define I2C_DIR_READ  1


typedef void (*i2c_recv_callback_t)(uint8_t, uint8_t);  // (offset, data)   -> ()
typedef uint8_t (*i2c_send_callback_t)(uint8_t);        // (offset)         -> (data)
typedef void (*i2c_stop_callback_t)();                  // ()               -> ()

typedef struct {
    i2c_recv_callback_t recv_cb;
    i2c_send_callback_t send_cb;
    i2c_stop_callback_t stop_cb;
    //i2c_error_callback_t error_cb;
} i2c_op_callbacks;



int i2c_slave_init(void);

void i2c_slave_reset(void);

void i2c_slave_set_callbacks(uint8_t cb_count, const i2c_op_callbacks *cb_array);

extern volatile uint32_t i2creqs;

#endif
