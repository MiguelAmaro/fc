#ifndef RINGBUFFER_H
#define RINGBUFFER_H
#include "LAL.h"

#define RING_BUFFER_DEFAULT_VALUE (' ')
#define RINGBUFFER_CAPACITY (256)

typedef struct
{
    u8  data[RINGBUFFER_CAPACITY];
    u16 size;
    u16 head;
    u16 tail;
} volatile RingBuffer;

// NOTE(MIGUEL): !!! Almost useless function !!!
void
RingBuffer_Init(RingBuffer *buffer);

b32 
RingBuffer_Full(RingBuffer* buffer);

b32 
RingBuffer_Empty(RingBuffer* buffer); 

void 
RingBuffer_Enqueue_Byte(RingBuffer *buffer, readonly u8 data);

u8 
RingBuffer_Dequeue_Byte(RingBuffer* buffer);


#endif // RINGBUFFER_H


