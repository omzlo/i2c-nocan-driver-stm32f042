#include <stm32f0xx.h>
#include "can.h"
#include "nocan.h"
#include "gpio.h"
#include "systick.h"
#include "spi_slave.h"
#include "chip_options.h"

nocan_registers_t NOCAN_REGS;
nocan_registers_t *NOCAN = &NOCAN_REGS;


#define MUX_COUNT 8
nocan_buffer_t nocan_mux[MUX_COUNT];
nocan_buffer_t *nocan_recv[MUX_COUNT+1];
uint8_t nocan_recv_head;
uint8_t nocan_recv_tail;

/***/
 
static void nocan_recv_push_back(nocan_buffer_t *buf)
{
    // Assert: buffer is never full
    buf->complete = 1;
    nocan_recv[nocan_recv_tail] = buf;
    nocan_recv_tail = (nocan_recv_tail+1) % MUX_COUNT;

    if (NOCAN->READ == 0)
        NOCAN->READ = buf->dlc+1+4;

    nocan_status_set(NOCAN_STATUS_RX_PENDING);
}

static nocan_buffer_t *nocan_recv_front(void)
{
    if (nocan_recv_head == nocan_recv_tail)
        return 0;
    return nocan_recv[nocan_recv_head];
}

static void nocan_recv_pop_front(void)
{
    nocan_buffer_t *buf = nocan_recv_front();
    if (buf)
    {
        buf->age = 0xFF;
        buf->complete = 0;        
        nocan_recv_head = (nocan_recv_head+1) % MUX_COUNT;
        if (nocan_recv_head == nocan_recv_tail)
        {
            nocan_status_clear(NOCAN_STATUS_RX_PENDING);
            NOCAN->READ = 0;
        }
        else
        {
            NOCAN->READ = nocan_recv[nocan_recv_head]->dlc+1+4;
        }
    }
}

/***/

static void nocan_mux_release_buffer(nocan_buffer_t *buffer)
{
    buffer->age = 0xFF;
    buffer->complete = 0;
}

static int mux_search(uint32_t eid)
{
    int i;
    uint8_t node_id = EID_GET_NODE_ID(eid);
    uint16_t parameters = EID_GET_CHANNEL_ID(eid);

    // Age all buffers, except empty ones which are marked with age==0xFF 
    // or the ones that are marked as complete. 
    // Eventually unused buffers go back to 0xFF (free state)
    for (i=0; i<MUX_COUNT; i++)
    {
        if (nocan_mux[i].age<0xFF && nocan_mux[i].complete==0) 
            nocan_mux[i].age++;
    }

    for (i=0; i<MUX_COUNT; i++)
    {
        if (nocan_mux[i].age<0xFF && nocan_mux[i].node_id==node_id)
        {
            if (nocan_mux[i].parameters == parameters)
            {
                nocan_mux[i].age = 0; // refresh
                return i;
            }
        }
    }

    // else: find any buffer that, either
    // - either is free (age==0xFF), 
    // - or has been there for the longest.
    int oldest = -1;
    uint8_t age = 0;
    for (i=0;i<MUX_COUNT;i++)
    {
        if (nocan_mux[i].age>age)
        {
            age = nocan_mux[i].age;
            oldest = i;
            if (age==0xFF) break;
        }
    }

    if (oldest>=0) {
        nocan_mux[oldest].age = 0;
        nocan_mux[oldest].node_id = node_id;
        nocan_mux[oldest].parameters = parameters;
        nocan_mux[oldest].dlc = 0;
        nocan_mux[oldest].eid[0] = (uint8_t)(eid>>24);
        nocan_mux[oldest].eid[1] = (uint8_t)(eid>>16);
        nocan_mux[oldest].eid[2] = (uint8_t)(eid>>8);
        nocan_mux[oldest].eid[3] = (uint8_t)(eid);
        return oldest;
    }

    return -1; 
}

int nocan_mux_process(const can_packet_t *msg)
{
    int bindex = mux_search(msg->eid);
    nocan_buffer_t *buf;

    if (bindex<0) 
        return NOCAN_NO_BUFFER_AVAILABLE;

    buf = nocan_mux + bindex;

    if (buf->dlc>56)
    {
        nocan_mux_release_buffer(buf);
        return NOCAN_ERROR;
    }

    if ((msg->eid&EID_FIRST_PACKET_MASK)!=0 && buf->dlc>0)
    {
        nocan_mux_release_buffer(buf);
        return NOCAN_ERROR;
    }

    uint8_t pos = buf->dlc;
    buf->data[pos++] = msg->data_l >> 24;
    buf->data[pos++] = msg->data_l >> 16;
    buf->data[pos++] = msg->data_l >> 8;
    buf->data[pos++] = msg->data_l;
    buf->data[pos++] = msg->data_h >> 24;
    buf->data[pos++] = msg->data_h >> 16;
    buf->data[pos++] = msg->data_h >> 8;
    buf->data[pos++] = msg->data_h;
    buf->dlc+=msg->dlc;

    if ((msg->eid&EID_LAST_PACKET_MASK)!=0)
    {
        nocan_recv_push_back(buf);
        return bindex;
    }
    return NOCAN_WORKING;
}

/******/

int nocan_sys_send(uint8_t node_id, uint8_t function, uint8_t param, const char *data, uint8_t dlen)
{
    while (!can_tx_empty());
    uint32_t eid = ((uint32_t)node_id<<21) | ((uint32_t)function<<8) | ((uint32_t)param) | EID_SYS_MASK;
    return can_tx_send(eid, data, dlen);
}

int nocan_sys_process(const can_packet_t *packet)
{
    int i;
    int8_t eid_param = (int8_t)EID_GET_PARAMETER(packet->eid);
    uint8_t eid_func = EID_GET_FUNCTION(packet->eid);

    if (eid_func == NOCAN_SYS_NODE_BOOT_REQUEST)
    {
        if ((eid_param&0x01)!=0)
        {
            gpio_write_reset(LOW);
            for (uint32_t count=0;count<10000;count++) asm("nop");
            gpio_write_reset(HIGH);
        }
        if ((eid_param&0x02)!=0)
        {
            NVIC_SystemReset();
        }
        return 0;
    }

    if (eid_func == NOCAN_SYS_NODE_PING)
    {
        uint8_t data[8];

        data[0] = packet->data_l >> 24;
        data[1] = packet->data_l >> 16;
        data[2] = packet->data_l >> 8;
        data[3] = packet->data_l;
        data[4] = packet->data_h >> 24;
        data[5] = packet->data_h >> 16;
        data[6] = packet->data_h >> 8;
        data[7] = packet->data_h;

        nocan_sys_send(NOCAN->NODE_ID,NOCAN_SYS_NODE_PING_ACK,eid_param+1,data,packet->dlc);
        return 0;
    }

    return nocan_mux_process(packet);
}

/* * * */

int nocan_status_set(uint8_t status)
{
    NOCAN->STATUS |= status;
    if ((status & NOCAN_STATUS_TX_PENDING)!=0)
    {
        gpio_write_can_tx_int(LOW);
    }
    if ((status & NOCAN_STATUS_RX_PENDING)!=0)
    {
        gpio_write_can_rx_int(LOW);
    }
    if ((status & NOCAN_STATUS_LED)!=0)
    {
        gpio_write_led(HIGH);
    }
}

int nocan_status_clear(uint8_t status)
{
    NOCAN->STATUS &= ~status;
    if ((status & NOCAN_STATUS_TX_PENDING)!=0)
    {
        gpio_write_can_tx_int(HIGH);
    }
    if ((status & NOCAN_STATUS_RX_PENDING)!=0)
    {
        gpio_write_can_rx_int(HIGH);
    }
    if ((status & NOCAN_STATUS_LED)!=0)
    {
        gpio_write_led(LOW);
    }
}

static void nocan_send(uint8_t *data)
{
    uint32_t eid = ((uint32_t)data[1]<<24) |
        ((uint32_t)data[2]<<16) |
        ((uint32_t)data[3]<<8) |
        ((uint32_t)data[4]);

    if (data[0]>=5) {
        can_tx_send(eid, data+6, data[5]);
    }
}

/* * * */

uint8_t op_data[1+4+1+64];
can_packet_t *op_packet;
nocan_buffer_t *op_buffer;

/* * SPI_OP_SYNC(0) * */

static void op_sync(uint8_t op)
{
    spi_send_byte(0xAC);   
}

/* * SPI_OP_GET_UDID(1) * */

static void op_get_udid(uint8_t op)
{
    spi_send_dma(8, NOCAN->UDID);
}

/* * SPI_OP_CLEAR_LED(2), SPI_OP_SET_LED(3) * */

static void op_set_led(uint8_t op)
{
    if (op&1)
       nocan_status_set(NOCAN_STATUS_LED);
    else
       nocan_status_clear(NOCAN_STATUS_LED);
    
    spi_send_byte(op&1);
}

/* * SPI_OP_SOFT_RESET(4), SPI_OP_HARD_RESET(5) * */

static void op_reset(uint8_t op)
{
    spi_send_byte(0xA0);

    if (op&1)
        NVIC_SystemReset();
    else
        nocan_init();
}

/* * SPI_OP_GET_STATUS(6) * */

static void op_get_status(uint8_t op)
{
    spi_send_dma(2, &(NOCAN->STATUS));
}

/* * SPI_OP_SEND(7) * */

static void op_send(uint8_t op)
{
    spi_send_byte(0xA1);
    
    op_data[0] = spi_recv_byte();

    if (op_data[0]>64+1+4)
        op_data[0] = 64+1+4;

    spi_recv_dma(op_data[0], op_data+1);

}

static void op_send_finalize(void)
{
    nocan_status_set(NOCAN_STATUS_TX_PENDING); 
    nocan_send(op_data);
}

/* * SPI_OP_RECV(8) * */

static void op_recv(uint8_t op)
{
    uint8_t rlen;

    spi_send_byte(0xA1);
    
    nocan_status_clear(NOCAN_STATUS_RX_ERROR);

    rlen = spi_recv_byte();

    if ((op_buffer = nocan_recv_front())!=0)
    {
        if (rlen>op_buffer->dlc+4+1)
            rlen = op_buffer->dlc+4+1;

        spi_send_dma(rlen, op_buffer->eid);
    }
}

static void op_recv_finalize(void)
{
    if (op_buffer)
        nocan_recv_pop_front();
}

/* * SPI_OP_RECV_MSG(9) * */

/* Not implemented */

/* * SPI_OP_MSG_FILTER_ADD(10) * */

static void op_msg_filter_add(uint8_t op)
{
    spi_send_byte(0xA1);
    
    spi_recv_dma(2, op_data);
}

static void op_msg_filter_add_finalize(void)
{
    can_msg_filter_channel_add(((uint16_t)op_data[0]<<8)|(uint16_t)op_data[1]);
}

/* * SPI_OP_MSG_FILTER_REM(11) * */

static void op_msg_filter_rem(uint8_t op)
{
    spi_send_byte(0xA1);

    spi_recv_dma(2, op_data);
}

static void op_msg_filter_rem_finalize(void)
{
    can_msg_filter_channel_remove(((uint16_t)op_data[0]<<8)|(uint16_t)op_data[1]);
}

/* * SPI_OP_SYS_FILTER_SET(12) * */

static void op_sys_filter_set(uint8_t op)
{
    uint8_t r;

    spi_send_byte(0xA1);

    r = spi_recv_byte();

    can_sys_filter_set(r);
}

/* * * */

static void op_nop(uint8_t op)
{
    /* Do nothing !*/
}
static void op_nop_finalize(void)
{
    /* Do nothing !*/
}

/* * * */

#define CB_COUNT 13
spi_callbacks_typedef op_table[CB_COUNT] = {
    [0] =   { op_sync,              op_nop_finalize },
    [1] =   { op_get_udid,          op_nop_finalize },
    [2] =   { op_set_led,           op_nop_finalize },
    [3] =   { op_set_led,           op_nop_finalize },
    [4] =   { op_reset,             op_nop_finalize },
    [5] =   { op_reset,             op_nop_finalize },
    [6] =   { op_get_status,        op_nop_finalize },
    [7] =   { op_send,              op_send_finalize },
    [8] =   { op_recv,              op_recv_finalize },
    [9] =   { op_nop,               op_nop_finalize },
    [10] =  { op_msg_filter_add,    op_msg_filter_add_finalize },
    [11] =  { op_msg_filter_rem,    op_msg_filter_rem_finalize },
    [12] =  { op_sys_filter_set,    op_nop_finalize },
};

uint8_t _compact_number(uint8_t n)
{
    if (n>='0' && n<='9') return n-'0';
    return 0xF;
}

void nocan_init()
{
    int i;
    uint8_t *p=(uint8_t *)NOCAN;

    can_init();

    nocan_recv_head = 0;
    nocan_recv_tail = 0;

    spi_slave_set_callbacks(CB_COUNT,op_table);

    for (i=0;i<MUX_COUNT;i++)
        nocan_mux_release_buffer(nocan_mux+i);

    for (i=0;i<sizeof(nocan_registers_t);i++)
        p[i]=0;

    nocan_status_clear(0xFF);

    NOCAN->UDID[0] = CHIP_UDID[0];
    NOCAN->UDID[1] = CHIP_UDID[2];
    NOCAN->UDID[2] = CHIP_UDID[4];
    NOCAN->UDID[3] = CHIP_UDID[5];
    NOCAN->UDID[4] = CHIP_UDID[6];
    NOCAN->UDID[5] = CHIP_UDID[7];
    NOCAN->UDID[6] = (_compact_number(CHIP_UDID[8])<<4) | _compact_number(CHIP_UDID[9]);
    NOCAN->UDID[7] = (_compact_number(CHIP_UDID[10])<<4) | _compact_number(CHIP_UDID[11]);
    NOCAN->GUARD = 0x42;

}

