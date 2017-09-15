#ifndef _CAN_H_
#define _CAN_H_

#include <stm32f0xx_can.h>

typedef struct __attribute__((packed)) {
    uint32_t eid;
    uint8_t dlc;
    uint8_t padding[3];
    uint8_t data[8];
} can_packet_t;

typedef struct {
    void (*receive_fifo_0_cb)(can_packet_t *);
    void (*receive_fifo_1_cb)(can_packet_t *);
    //void (*send_complete_0_cb)(void);
} can_configuration_t;

int can_init(const can_configuration_t *callbacks);

int can_filter_add(uint32_t can_id, uint32_t mask);

int can_filter_modify(int filter_id, uint32_t can_id, uint32_t mask);

int can_filter_remove(int filter_id);

int can_send(const can_packet_t *packet);

//int can_tx_ready(void);

#endif
