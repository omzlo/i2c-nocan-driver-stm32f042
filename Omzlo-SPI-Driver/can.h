#ifndef _CAN_H_
#define _CAN_H_

#include <stdint.h>

typedef struct __attribute__((packed)) {
    uint32_t eid;
    uint32_t dlc;
    uint32_t data_l;
    uint32_t data_h;
} can_packet_t;

int can_init(void);

int can_msg_filter_channel_add(uint16_t channelid);

int can_msg_filter_channel_remove(uint16_t channelid);

int can_sys_filter_set(uint8_t nodeid);

int can_tx_empty(void);

int can_tx_send(uint32_t eid, const uint8_t *data, uint8_t dlen);

#endif
