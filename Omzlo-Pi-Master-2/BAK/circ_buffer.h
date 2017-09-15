#ifndef CIRC_BUFFER_H
#define CIRC_BUFFER_H

#include <stdint.h>

typedef struct {
    uint16_t slot_size;
    uint16_t slot_count;
    uint16_t head;
    uint16_t tail;
    uint8_t *data;
} circ_buffer_t;

void circ_buffer_init(circ_buffer_t *cbuf, uint16_t slot_size, uint16_t slot_count, void *data);

int circ_buffer_enqueue(circ_buffer_t *cbuf, const void *data);

int circ_buffer_shift(circ_buffer_t *cbuf);

void *circ_buffer_head(circ_buffer_t *cbuf);

int circ_buffer_full(circ_buffer_t *cbuf);

int circ_buffer_empty(circ_buffer_t *cbuf);

#endif
