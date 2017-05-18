#ifndef _CAN_H_
#define _CAN_H_

#include <stm32f0xx_can.h>

typedef struct __attribute__((packed)) {
    uint32_t eid;
    uint8_t dlc;
    uint8_t padding[3];
    uint8_t data[8];
} can_packet_t;

int can_init(void);

int can_msg_filter_channel_add(uint16_t channelid);

int can_msg_filter_channel_remove(uint16_t channelid);

int can_sys_filter_set(uint8_t nodeid);

int can_tx_empty(void);

int can_tx_send(uint32_t eid, const uint8_t *data, uint8_t dlen);

#endif
