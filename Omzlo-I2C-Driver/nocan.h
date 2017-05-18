#ifndef _NOCAN_H_
#define _NOCAN_H_

#include "can.h"

/*
    EID format used:
    pos size    description
    ------------------------
    0   1       first_packet_flag   (0=>not the fist packet in a block, 1=> is the first packet)
    1   7       node_id      
    8   1       last_packet_flag    (0=>not the last packet in a block, 1=> is not the last packet)
    9   1       reserved 
    10  1       sys_flag

    -- if sys_flag == 0 then
    11  2       reserved
    13  16      channel_id 

    -- if sys_flag == 1 then
    11  2       reserved
    13  8       function
    21  8       parameter
       
    ------------------------
    29  -       total length
*/ 

#define EID_FIRST_PACKET_MASK       (1<<28)
#define EID_NODE_ID_MASK            (0x7F<<21)
#define EID_LAST_PACKET_MASK        (1<<20)
#define EID_SYS_MASK                (1<<18)
#define EID_CHANNEL_ID_MASK         (0xFFFF)
#define EID_FUNCTION_MASK           (0xFF00)
#define EID_PARAMETER_MASK          (0x00FF)

#define EID_GET_NODE_ID(eid)        (((eid)&EID_NODE_ID_MASK)>>21)
#define EID_GET_CHANNEL_ID(eid)     (((eid)&EID_CHANNEL_ID_MASK))
#define EID_GET_FUNCTION(eid)       (((eid)&EID_FUNCTION_MASK)>>8)
#define EID_GET_PARAMETER(eid)      (((eid)&EID_PARAMETER_MASK))

typedef struct __attribute__((packed)) {
    uint8_t age;
    uint8_t node_id;
    uint16_t channel_id;
    uint8_t dlen;
    uint8_t complete;
    uint8_t data[64];
} nocan_buffer_t;

#define NOCAN_SYS_ANY                      0
#define NOCAN_SYS_ADDRESS_REQUEST          1
#define NOCAN_SYS_ADDRESS_CONFIGURE        2
#define NOCAN_SYS_ADDRESS_CONFIGURE_ACK    3
#define NOCAN_SYS_ADDRESS_LOOKUP           4
#define NOCAN_SYS_ADDRESS_LOOKUP_ACK       5
#define NOCAN_SYS_NODE_BOOT_REQUEST        6
#define NOCAN_SYS_NODE_BOOT_ACK            7
#define NOCAN_SYS_NODE_PING                8
#define NOCAN_SYS_NODE_PING_ACK            9

#define NOCAN_SYS_CHANNEL_REGISTER           10
#define NOCAN_SYS_CHANNEL_REGISTER_ACK       11
#define NOCAN_SYS_CHANNEL_UNREGISTER         12
#define NOCAN_SYS_CHANNEL_UNREGISTER_ACK     13

#define NOCAN_SYS_CHANNEL_SUBSCRIBE          14
#define NOCAN_SYS_CHANNEL_UNSUBSCRIBE        15
#define NOCAN_SYS_CHANNEL_LOOKUP             16
#define NOCAN_SYS_CHANNEL_LOOKUP_ACK         17

#define NOCAN_SYS_BOOTLOADER_GET_SIGNATURE     18
#define NOCAN_SYS_BOOTLOADER_GET_SIGNATURE_ACK 19
#define NOCAN_SYS_BOOTLOADER_SET_ADDRESS       20
#define NOCAN_SYS_BOOTLOADER_SET_ADDRESS_ACK   21
#define NOCAN_SYS_BOOTLOADER_WRITE         22
#define NOCAN_SYS_BOOTLOADER_WRITE_ACK     23
#define NOCAN_SYS_BOOTLOADER_READ          24
#define NOCAN_SYS_BOOTLOADER_READ_ACK      25
#define NOCAN_SYS_BOOTLOADER_LEAVE         26
#define NOCAN_SYS_BOOTLOADER_LEAVE_ACK     27


#define NOCAN_ERROR                 -1
#define NOCAN_NO_DATA               -2
#define NOCAN_NO_BUFFER_AVAILABLE   -3
#define NOCAN_WORKING               -16

void nocan_init(void);

//void nocan_mux_release_buffer(int index);

int nocan_mux_process(const can_packet_t *msg);

int nocan_sys_send(uint8_t node_id, uint8_t function, uint8_t param, const char *data, uint8_t dlen);

int nocan_sys_process(const can_packet_t *msg);

int nocan_status_set(uint8_t status);

int nocan_status_clear(uint8_t status);

#define NOCAN_STATUS_RX_MASK  0x07
#define NOCAN_STATUS_RX_SYS   0x01 
#define NOCAN_STATUS_RX_MSG   0x02
#define NOCAN_STATUS_RX_ERR   0x04
#define NOCAN_STATUS_TX_RDY   0x08
#define NOCAN_STATUS_TX_ERR   0x10

#define NOCAN_PSTATUS_INT      0x40
#define NOCAN_PSTATUS_LED      0x80


typedef struct __attribute__((packed)) {
    volatile uint8_t STATUS;    // status register
    volatile uint8_t PSTATUS;   // pin status register
    uint8_t NODE_ID;
    uint8_t UDID[8];
    uint8_t GUARD;
    volatile uint32_t I2C_START_COUNT;
    volatile uint32_t I2C_STOP_COUNT;
    volatile uint32_t CAN_RX_COUNT;
    volatile uint32_t CAN_TX_COUNT;
} nocan_registers_t;

extern nocan_registers_t *NOCAN;

#endif
