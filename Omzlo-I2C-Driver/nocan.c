#include "can.h"
#include "nocan.h"
#include "gpio.h"
#include "systick.h"
#include "i2c_slave.h"
#include "chip_options.h"

nocan_registers_t NOCAN_REGS;
nocan_registers_t *NOCAN = &NOCAN_REGS;


#define MUX_COUNT 8
nocan_buffer_t nocan_mux[MUX_COUNT];
volatile unsigned nocan_complete;

#define SYS_COUNT 4
can_packet_t nocan_sys_queue[SYS_COUNT];
uint8_t nocan_sys_queue_head;
uint8_t nocan_sys_queue_tail;



void nocan_mux_release_buffer(nocan_buffer_t *buffer)
{
    if (buffer->complete)
    {
        nocan_complete--;
        if (nocan_complete==0)
            nocan_status_clear(NOCAN_STATUS_RX_MSG);
    }
    buffer->age = 0xFF;
    buffer->complete = 0;
}

static int mux_search(uint8_t node_id, uint16_t channel_id)
{
    int i;

    // Age all buffers, except empty ones which are marked with age==0xFF 
    // Eventually unused buffers go back to 0xFF (free state)
    for (i=0; i<MUX_COUNT; i++)
        if (nocan_mux[i].age<0xFF) nocan_mux[i].age++;

    for (i=0; i<MUX_COUNT; i++)
    {
        if (nocan_mux[i].age<0xFF && nocan_mux[i].node_id==node_id)
        {
            if (nocan_mux[i].channel_id == channel_id)
            {
                nocan_mux[i].age = 0; // refresh
                return i;
            }
            else // should not happen, but we clean just in case
            {
                nocan_mux_release_buffer(nocan_mux+i);
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
        nocan_mux[oldest].channel_id = channel_id;
        nocan_mux[oldest].dlen = 0;
        return oldest;
    }

    return -1; 
}

int nocan_mux_process(const can_packet_t *msg)
{
    if ((msg->eid&EID_SYS_MASK)!=0)
        return NOCAN_NO_DATA;

    int bindex = mux_search(EID_GET_NODE_ID(msg->eid), EID_GET_CHANNEL_ID(msg->eid));
    
    if (bindex<0) 
        return NOCAN_NO_BUFFER_AVAILABLE;

    if (nocan_mux[bindex].dlen>56)
    {
        nocan_mux_release_buffer(nocan_mux+bindex);
        return NOCAN_ERROR;
    }

    if ((msg->eid&EID_FIRST_PACKET_MASK)!=0 && nocan_mux[bindex].dlen>0)
    {
        nocan_mux_release_buffer(nocan_mux+bindex);
        return NOCAN_ERROR;
    }

    for (int i=0; i<msg->dlc; i++)
    {
        nocan_mux[bindex].data[nocan_mux[bindex].dlen++] = msg->data[i];
    }

    if ((msg->eid&EID_LAST_PACKET_MASK)!=0)
    {
        nocan_mux[bindex].complete = 1;
        nocan_complete++;
        nocan_status_set(NOCAN_STATUS_RX_MSG);
        return bindex;
    }
    return NOCAN_WORKING;
}

nocan_buffer_t *nocan_mux_head(void)
{
    if (nocan_complete==0)
        return 0;

    for (int i=0; i<MUX_COUNT; i++)
    {
        if (nocan_mux[i].complete) return nocan_mux+i;
    }
    return 0;
}

int nocan_sys_send(uint8_t node_id, uint8_t function, uint8_t param, const char *data, uint8_t dlen)
{
    while (!can_tx_empty());
    uint32_t eid = ((uint32_t)node_id<<21) | ((uint32_t)function<<8) | ((uint32_t)param) | EID_SYS_MASK;
    return can_tx_send(eid, data, dlen);
}

int nocan_sys_enqueue(const can_packet_t *packet)
{
    if (((nocan_sys_queue_tail+1)&(SYS_COUNT-1))==nocan_sys_queue_head)
        return -1;
    nocan_sys_queue[nocan_sys_queue_tail] = *packet;
    nocan_sys_queue_tail = (nocan_sys_queue_tail+1)&(SYS_COUNT-1);
    nocan_status_set(NOCAN_STATUS_RX_SYS);
    return 0;
}

void nocan_sys_dequeue()
{
    if (nocan_sys_queue_tail!=nocan_sys_queue_head)
        nocan_sys_queue_head = ((nocan_sys_queue_head+1)&(SYS_COUNT-1));
    if (nocan_sys_queue_tail==nocan_sys_queue_head)
        nocan_status_clear(NOCAN_STATUS_RX_SYS);
}

can_packet_t *nocan_sys_head()
{
    if (nocan_sys_queue_tail==nocan_sys_queue_head)
        return 0;
    return nocan_sys_queue+nocan_sys_queue_head;
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
        nocan_sys_send(NOCAN->NODE_ID,NOCAN_SYS_NODE_PING_ACK,eid_param+1,packet->data,packet->dlc);
        return 0;
    }

    return nocan_sys_enqueue(packet);
}

/* * * */

int nocan_status_set(uint8_t status)
{
    NOCAN->STATUS |= status;
    if ((NOCAN->STATUS & NOCAN_STATUS_RX_MASK)!=0)
    {
        gpio_write_can_int(LOW);
        NOCAN->PSTATUS &= ~NOCAN_PSTATUS_INT;
    }
}

int nocan_status_clear(uint8_t status)
{
    NOCAN->STATUS &= ~status;
    if ((NOCAN->STATUS & NOCAN_STATUS_RX_MASK)==0)
    {
        gpio_write_can_int(HIGH);
        NOCAN->PSTATUS |= NOCAN_PSTATUS_INT;
    }
}

/* * * */

uint8_t op_state;
//uint8_t op_addr;
uint32_t op_eid;
uint8_t op_dlen;
uint8_t op_data[64];
can_packet_t *op_packet;
nocan_buffer_t *op_buffer;

#define OP_STATE_INIT       0
//#define OP_STATE_PARAM1     1
//#define OP_STATE_PARAM2     2
//#define OP_STATE_PARAM3     3
//#define OP_STATE_ERROR      254
#define OP_STATE_DONE       255

/* * * */

//static void op_start_nop(uint8_t op, uint8_t dir)
//{
    // void
//}

static void op_recv_nop(uint8_t offset, uint8_t data)
{
    // void
}

static uint8_t op_send_nop(uint8_t offset)
{
    return 0xEE;
}

static void op_stop_nop(void)
{
    // void
}

/* * TWI_OP_GET_UDID * */

//static void op_start_get_udid(uint8_t op, uint8_t dir)
//{
//    if (dir==I2C_DIR_READ)
//    {
//        op_addr = 0;
//    }
//}

static uint8_t op_send_get_udid(uint8_t offset)
{
    return NOCAN->UDID[offset&7];
}

/* * TWI_OP_SET_LED * */

static void op_recv_set_led(uint8_t offset, uint8_t data)
{
    if (data)
        gpio_write_can_led(HIGH);
    else
        gpio_write_can_led(LOW);
}

/* * TWI_OP_RESET * */

static void op_recv_reset(uint8_t offset, uint8_t data)
{
    if (offset==1)
    {
        if (data==1)
            nocan_init();
    }
}

/* * TWI_OP_GET_STATUS * */

static uint8_t op_send_get_status(uint8_t offset)
{
    return NOCAN->STATUS;
}

/* * TWI_OP_SEND * */

//void op_start_send(uint8_t op, uint8_t dir)
//{
//    op_addr = 0;
//    op_dlen = 0;
//}

static void op_recv_send(uint8_t offset, uint8_t data)
{
    switch (offset) {
        case 0:
            op_dlen = 0;
            op_state = OP_STATE_INIT;
            break;
        case 1: 
            op_eid = (uint32_t)data<<24; 
            break;
        case 2:
            op_eid |= (uint32_t)data<<16; 
            break;
        case 3: 
            op_eid |= (uint32_t)data<<8; 
            break;
        case 4: 
            op_eid |= (uint32_t)data; 
            break;
        case 5: 
            op_state = OP_STATE_DONE;
            if (data>64) 
                op_dlen = 64;
            else
                op_dlen = data;
            break;
        default: // 6 or more
            if (offset-6<op_dlen)
                op_data[offset-6] = data;
    }
}

static void op_stop_send(void)
{
    nocan_status_clear(NOCAN_STATUS_TX_RDY | NOCAN_STATUS_TX_ERR); 
    if (op_state == OP_STATE_DONE)
    {
        can_tx_send(op_eid, op_data, op_dlen);
    }
}

/* * TWI_OP_RECV_SYS * */

//static void op_start_recv_sys(uint8_t op, uint8_t dir)
//{
//    if (dir==I2C_DIR_READ)
//    {
//        op_addr = 0;
//        op_packet = nocan_sys_head();
//
//    }
//    else
//    {
//        nocan_status_clear(NOCAN_STATUS_RX_ERR); 
//    }
//}

static void op_recv_recv_sys(uint8_t offset, uint8_t data)
{
    nocan_status_clear(NOCAN_STATUS_RX_ERR);
}

static uint8_t op_send_recv_sys(uint8_t offset)
{
    uint8_t r;

    if (offset==0)
        op_packet = nocan_sys_head();

    if (op_packet==0)
        return 0;

    if (offset>op_packet->dlc+5)
        return 0;

    switch (offset) {
        case 0:
           r = (op_packet->eid>>24)&0xFF;
           break;
        case 1:
           r = (op_packet->eid>>16)&0xFF;
           break;
        case 2:
           r = (op_packet->eid>>8)&0xFF;
           break;
        case 3:
           r = op_packet->eid&0xFF;
           break;
        case 4: 
           r = op_packet->dlc;
           break;
        default: // 5 or more
           if (offset-5<op_packet->dlc)
               r = op_packet->data[offset-5];
           else
               r = 0x00;    // filler
    }
    return r;
}

static void op_stop_recv_sys(void)
{
    if (op_packet==0)
        return;

    nocan_sys_dequeue();
}

/* * TWI_OP_RECV_MSG * */

//static void op_start_recv_msg(uint8_t op, uint8_t dir)
//{
//    if (dir==I2C_DIR_READ)
//    {
//        op_addr = 0;
//        op_buffer = nocan_mux_head();
//    }
//    else
//    {
//        nocan_status_clear(NOCAN_STATUS_RX_ERR); 
//    }
//}

static void op_recv_recv_msg(uint8_t offset, uint8_t data)
{
    nocan_status_clear(NOCAN_STATUS_RX_ERR);
}


static uint8_t op_send_recv_msg(uint8_t offset)
{
    uint8_t r;

    if (offset==0)
        op_buffer = nocan_mux_head();

    if (op_buffer==0)
        return 0;

    if (offset>op_buffer->dlen+5)
        return 0;

    switch (offset) {
        case 0:
           r = (op_buffer->node_id>>3)&0xFF;
           break;
        case 1:
           r = (op_buffer->node_id<<5)&0xFF;
           break;
        case 2:
           r = (op_buffer->channel_id>>8)&0xFF;
           break;
        case 3:
           r = op_buffer->channel_id&0xFF;
           break;
        case 4: 
           r = op_buffer->dlen;
           break;
        default:
           if (offset-5<op_buffer->dlen)
               r = op_buffer->data[offset-5];
           else
               r = 0x00; // filler
    } 
    return r;
}

static void op_stop_recv_msg(void)
{
    if (op_buffer==0)
        return;

    nocan_mux_release_buffer(op_buffer);
}

/* * TWI_OP_MSG_FILTER_ADD * */
/*
static void op_start_filter_add(uint8_t op, uint8_t dir)
{
    if (dir==I2C_DIR_WRITE)
    {
        op_state = OP_STATE_PARAM1;
    }
    else
    {
        if (op_state == OP_STATE_DONE)
        {
            if (!can_msg_filter_channel_add(((uint16_t)op_data[0]<<8)|(uint16_t)op_data[1]))
                op_state == OP_STATE_ERROR;
        }
    }
}
*/

static void op_recv_filter_add(uint8_t offset, uint8_t data)
{
    if (offset==0)
    {
        op_state = OP_STATE_INIT;
    }
    else if (offset==1)
    {
        op_data[0] = data;
    }
    else if (offset==2)
    {
        op_data[1] = data;
        op_state = OP_STATE_DONE;
    }
}

static uint8_t op_send_filter_add(uint8_t offset)
{
    if (offset!=0)
        return 3;

    if (op_state == OP_STATE_DONE)
    {
        if (!can_msg_filter_channel_add(((uint16_t)op_data[0]<<8)|(uint16_t)op_data[1]))
            return 1;
        return 0;
    }
    return 2;
}

/* * TWI_OP_MSG_FILTER_REM * */
/*
static void op_start_filter_rem(uint8_t op, uint8_t dir)
{
    if (dir==I2C_DIR_WRITE)
    {
        op_state = OP_STATE_PARAM1;
    }
    else
    {
        if (op_state == OP_STATE_DONE)
        {
            if (!can_msg_filter_channel_remove(((uint16_t)op_data[0]<<8)|(uint16_t)op_data[1]))
                op_state == OP_STATE_ERROR;
        }
    }
}
*/

static void op_recv_filter_rem(uint8_t offset, uint8_t data)
{
    if (offset==0)
    {
        op_state = OP_STATE_INIT;
    }
    else if (offset==1)
    {
        op_data[0] = data;
    }
    else if (offset==2)
    {
        op_data[1] = data;
        op_state = OP_STATE_DONE;
    }
}

static uint8_t op_send_filter_rem(uint8_t offset)
{
    if (offset!=0)
        return 3;

    if (op_state == OP_STATE_DONE)
    {
        if (!can_msg_filter_channel_remove(((uint16_t)op_data[0]<<8)|(uint16_t)op_data[1]))
            return 1;
        return 0;
    }
    return 2;
}

/* * TWI_OP_SYS_FILTER_SET * */
/*
static void op_start_filter_set(uint8_t op, uint8_t dir)
{
    if (dir==I2C_DIR_WRITE)
    {
        op_state = OP_STATE_PARAM1;
    }
    else
    {
        if (op_state == OP_STATE_DONE)
        {
            if (!can_sys_filter_set(op_data[0]))
                op_state = OP_STATE_ERROR;
        }
    }
}
*/

static void op_recv_filter_set(uint8_t offset, uint8_t data)
{
    if (offset==0)
    {
        op_state = OP_STATE_INIT;
    }
    else if (offset==1)
    {
        op_data[0] = data;
        op_state = OP_STATE_DONE;
    }
}

static uint8_t op_send_filter_set(uint8_t offset)
{
    if (offset!=0)
        return 3;

    if (op_state == OP_STATE_DONE)
    {
        if (!can_sys_filter_set(op_data[0]))
            return 1;
        return 0;
    }
    return 2;
}

/* * * */

#define CB_COUNT 11
i2c_op_callbacks op_table[CB_COUNT] = {
    [0] = { op_recv_nop,        op_send_nop,        op_stop_nop }, 
    [1] = { op_recv_set_led,    op_send_nop,        op_stop_nop },
    [2] = { op_recv_reset,      op_send_nop,        op_stop_nop },
    [3] = { op_recv_nop,        op_send_get_status, op_stop_nop },
    [4] = { op_recv_send,       op_send_nop,        op_stop_send },
    [5] = { op_recv_recv_sys,   op_send_recv_sys,   op_stop_recv_sys },
    [6] = { op_recv_recv_msg,   op_send_recv_msg,   op_stop_recv_msg },
    [7] = { op_recv_filter_add, op_send_filter_add, op_stop_nop },
    [8] = { op_recv_filter_rem, op_send_filter_rem, op_stop_nop },
    [9] = { op_recv_filter_set, op_send_filter_set, op_stop_nop },
    [10]= { op_recv_nop,        op_send_get_udid,   op_stop_nop }, 
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

    i2c_slave_set_callbacks(CB_COUNT,op_table);

    for (i=0;i<MUX_COUNT;i++)
        nocan_mux_release_buffer(nocan_mux+i);
    nocan_complete = 0;

    for (i=0;i<sizeof(nocan_registers_t);i++)
        p[i]=0;

    nocan_status_clear(0xFF);
    gpio_write_can_led(LOW);

    NOCAN->STATUS  = NOCAN_STATUS_TX_RDY;
    NOCAN->UDID[0] = CHIP_UDID[0];
    NOCAN->UDID[1] = CHIP_UDID[2];
    NOCAN->UDID[2] = CHIP_UDID[4];
    NOCAN->UDID[3] = CHIP_UDID[5];
    NOCAN->UDID[4] = CHIP_UDID[6];
    NOCAN->UDID[5] = CHIP_UDID[7];
    NOCAN->UDID[6] = (_compact_number(CHIP_UDID[8])<<4) | _compact_number(CHIP_UDID[9]);
    NOCAN->UDID[7] = (_compact_number(CHIP_UDID[10])<<4) | _compact_number(CHIP_UDID[11]);
    NOCAN->GUARD = 0x42;

    nocan_sys_queue_head = 0;
    nocan_sys_queue_tail = 0;
}

