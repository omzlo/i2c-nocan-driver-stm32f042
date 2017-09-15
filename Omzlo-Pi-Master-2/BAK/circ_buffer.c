#include "circ_buffer.h"

static void ucpy(uint8_t *dst, const uint8_t *src, unsigned n)
{
    while (n--) *dst++=*src++;
}

void circ_buffer_init(circ_buffer_t *cbuf, uint16_t slot_size, uint16_t slot_count, void *data)
{
    cbuf->slot_size  = slot_size;
    cbuf->slot_count = slot_count;
    cbuf->head = 0;
    cbuf->tail = 0;
    cbuf->data = (uint8_t *)data;
}

int circ_buffer_enqueue(circ_buffer_t *cbuf, const void *data)
{
    if (circ_buffer_full(cbuf))
        return 0;
    ucpy(cbuf->data+cbuf->tail*cbuf->slot_size,(const uint8_t*)data,cbuf->slot_size);
    cbuf->tail = (cbuf->tail+1)%cbuf->slot_count;
    return 1;
}

int circ_buffer_shift(circ_buffer_t *cbuf)
{
    if (circ_buffer_empty(cbuf))
        return 0;
    cbuf->head = (cbuf->head+1)%cbuf->slot_count;
    return 1;
}

void *circ_buffer_head(circ_buffer_t *cbuf)
{
    return cbuf->data+cbuf->head*cbuf->slot_size;
}

int circ_buffer_full(circ_buffer_t *cbuf)
{
    return ((cbuf->tail+1)%cbuf->slot_count)==cbuf->head;
}

int circ_buffer_empty(circ_buffer_t *cbuf)
{
    return cbuf->tail==cbuf->head;
}



