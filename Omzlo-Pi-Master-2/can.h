#ifndef _CAN_H_
#define _CAN_H_

#include <stdint.h>

typedef struct __attribute__((packed)) {
    uint8_t eid[4];
    uint8_t dlc;
    uint8_t data[8];
} can_packet_typedef;

int can_init(void);

int can_filter_add(uint32_t can_id, uint32_t mask);

int can_filter_modify(int filter_id, uint32_t can_id, uint32_t mask);

int can_filter_remove(int filter_id);

int can_send(const can_packet_typedef *packet);

//int can_tx_ready(void);

#endif
