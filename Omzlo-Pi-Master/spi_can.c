#include "spi_can.h"
#include <stdint.h>
#include "gpio.h"
#include "can.h"
#include "circ_buffer.h"
#include "spi_slave.h"
#include "power.h"

nocan_registers_t NOCAN_REGS;
nocan_registers_t *NOCAN = &NOCAN_REGS;

//#define SEND_BUFFER_SIZE 8
#define RECV_BUFFER_SIZE 16
//can_packet_t _send_buffer[SEND_BUFFER_SIZE];
can_packet_t _recv_buffer[RECV_BUFFER_SIZE];

//circ_buffer_t can_send_buffer;
circ_buffer_t can_recv_buffer;

/*
static void can_send_complete_cb(void)
{
    if (!circ_buffer_empty(&can_send_buffer))
    {
        can_send(circ_buffer_head(&can_send_buffer));
        circ_buffer_shift(&can_send_buffer);
        gpio_set_CAN_TX_INT(1);
    }
}

static int can_send_enqueue(const can_packet_t *packet)
{
    if (circ_buffer_enqueue(&can_send_buffer, packet))
    {
        if (circ_buffer_full(&can_send_buffer))
            gpio_set_CAN_TX_INT(0);
        if (can_tx_ready())
            can_send_complete_cb();
        return 1;
    }
    return 0;
}
*/

static can_packet_t *can_recv_head(void)
{
    return (can_packet_t *)circ_buffer_head(&can_recv_buffer);
}

static int can_recv_shift(void)
{
    int r = circ_buffer_shift(&can_recv_buffer);

    if (circ_buffer_empty(&can_recv_buffer))
    {
        gpio_set_CAN_RX_INT(1);
    }

    return r;
}

static void can_receive_complete_cb(can_packet_t *packet)
{
    circ_buffer_enqueue(&can_recv_buffer, packet);
    gpio_set_CAN_RX_INT(0);
}

/* 0 */

static uint8_t op_nop(uint8_t pos, uint8_t val)
{
    return 0xFF;
}

/* 1 */

static uint8_t op_reset(uint8_t pos, uint8_t val)
{
    if (pos==0)
        return 0;

    if (pos==1 && val==1)
    {
        spi_can_init(INIT_PHASE_DATA);
        spi_can_init(INIT_PHASE_SPI);
        spi_can_init(INIT_PHASE_CAN);
        return 1;
    }
    return 0;
}

/* 2 */

uint8_t offset = 0;

static uint8_t op_read_registers(uint8_t pos, uint8_t val)
{
    if (pos==0)
        return 0;

    if (pos==1)
        offset = val;
    
    return ((uint8_t *)NOCAN)[(pos-1+offset)%sizeof(*NOCAN)];
}


/* 3 */

static uint8_t op_set_power(uint8_t pos, uint8_t val)
{
    if (pos==1)
    {
        if (val==0)
            power_off();
        else
            power_on();
        return 1;
    }
    return 0;
}

/* 4 */

static uint8_t op_set_can_res(uint8_t pos, uint8_t val)
{
    return val;
}

/* 5 */

static uint8_t op_get_status(uint8_t pos, uint8_t val)
{
    return NOCAN->STATUS;
}

/* 6 */

can_packet_t spacket;

static uint8_t op_send_packet(uint8_t pos, uint8_t val)
{
    switch (pos) {
        case 0:
            return 0;
        case 1:
            spacket.eid = (uint32_t)val<<24;
            return 0;
        case 2:
            spacket.eid |= (uint32_t)val<<16;
            return 0;
         case 3:
            spacket.eid |= (uint32_t)val<<8;
            return 0;
         case 4:
            spacket.eid |= (uint32_t)val;
            return 0;
         case 5:
            spacket.dlc = val;
            return 0;
         case 6:    // 0
         case 7:    // 1
         case 8:    // 2
         case 9:    // 3
         case 10:   // 4
         case 11:   // 5
         case 12:   // 6
            spacket.data[pos-6] = val;
            return pos;
         case 13:   // 7
            spacket.data[7] = val;
            return can_send(&spacket)==0;
    }
    return 0xEE;
}

/* 7 */

can_packet_t *rpacket;

static uint8_t op_recv_packet(uint8_t pos, uint8_t val)
{
    uint8_t r;

    if (pos==0)
    {
        rpacket = can_recv_head();
        return rpacket!=0;
    }

    if (rpacket==0)
        return 0;


    switch (pos) {
        case 1:
            return (rpacket->eid>>24)&0xFF;
        case 2:
            return (rpacket->eid>>16)&0xFF;
        case 3:
            return (rpacket->eid>>8)&0xFF;
        case 4:
            return rpacket->eid&0xFF;
        case 5:
            return rpacket->dlc;
        case 6:    // 0
        case 7:    // 1
        case 8:    // 2
        case 9:    // 3
        case 10:   // 4
        case 11:   // 5
        case 12:   // 6
            return rpacket->data[pos-6];
        case 13:   // 7
            r =  rpacket->data[7]; 
            can_recv_shift();
            return r;
    }
    return 1;
}

/*****/


#define CB_COUNT 8
const spi_transfer_cb op_table[CB_COUNT] = {
    op_nop,
    op_reset,
    op_read_registers,
    op_set_power,
    op_set_can_res,
    op_get_status,
    op_send_packet,
    op_recv_packet,
};

const can_configuration_t can_config = {
    .receive_fifo_0_cb = can_receive_complete_cb,
    .receive_fifo_1_cb = can_receive_complete_cb,
    //.send_complete_0_cb = can_send_complete_cb,
};


const uint8_t *CHIP_UDID = (const uint8_t *)0x1FFFF7AC;
void spi_can_init(int phase) 
{
    int i;

    switch (phase) {
        case INIT_PHASE_SPI:
            spi_slave_set_callbacks(CB_COUNT,op_table);
            break;

        case INIT_PHASE_DATA:
            // set buffers for can
            //circ_buffer_init(&can_send_buffer,sizeof(can_packet_t),SEND_BUFFER_SIZE,_send_buffer);
            circ_buffer_init(&can_recv_buffer,sizeof(can_packet_t),RECV_BUFFER_SIZE,_recv_buffer);

            // reset registers
            for (i=0; i<sizeof(NOCAN_REGS);i++) ((uint8_t *)NOCAN)[i]=0;

            NOCAN_REGS.SIGNATURE[0] = 'C';
            NOCAN_REGS.SIGNATURE[1] = 'A';
            NOCAN_REGS.SIGNATURE[2] = 'N';
            NOCAN_REGS.SIGNATURE[3] = '0';
            NOCAN_REGS.VERSION_MAJOR = 0;
            NOCAN_REGS.VERSION_MINOR = 1;
            for (i=0;i<12;i++) NOCAN_REGS.CHIP_UDID[i]=CHIP_UDID[i];
            NOCAN_REGS.GUARD = 0xBEEF;
            break;

        case INIT_PHASE_CAN:
            // reset gpios to high
            gpio_set_CAN_TX_INT(1);
            gpio_set_CAN_RX_INT(1);
            // set callbacks for can
            //config.receive_fifo_0_cb = can_receive_complete_cb;
            //config.receive_fifo_1_cb = can_receive_complete_cb;
            //config.send_complete_0_cb = can_send_complete_cb;
            can_init(&can_config);
            // can_init includes a can_filter_add(0,0);
            break;
    }
}

