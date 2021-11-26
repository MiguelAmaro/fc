#include "RingBuffer.h"
#include <MK82F25615.h>

void 
RingBuffer_Init(RingBuffer *buffer) 
{
    buffer->size     = 0;
    buffer->head     = 0;
    buffer->tail     = 0;
    
    for(u32 data_index = 0; data_index < RINGBUFFER_CAPACITY; data_index++)
    {
        buffer->data[data_index] = RING_BUFFER_DEFAULT_VALUE;
    }
    
    return;
}

b32 
RingBuffer_Full(RingBuffer* buffer) 
{
    
    return buffer->size == RINGBUFFER_CAPACITY;
}

b32 
RingBuffer_Empty(RingBuffer* buffer) 
{
    
    return buffer->size == 0;
}

void 
RingBuffer_Enqueue_Byte(RingBuffer *buffer, readonly u8 data) 
{
    u32 masking_state;
    
    if(!RingBuffer_Full(buffer))
    {
        buffer->data[buffer->tail++] = data;
        buffer->tail %= RINGBUFFER_CAPACITY;
        masking_state = __get_PRIMASK();
        __disable_irq();
        buffer->size++;
        __set_PRIMASK(masking_state);
    }
    
    return;
}

u8 
RingBuffer_Dequeue_Byte(RingBuffer* buffer) 
{
    u32 masking_state;
    u8 data = 0;
    
    if(!RingBuffer_Empty(buffer))
    {
        data = buffer->data[buffer->head];
        buffer->data[buffer->head++] = RING_BUFFER_DEFAULT_VALUE;
        buffer->head %= RINGBUFFER_CAPACITY;
        masking_state = __get_PRIMASK();
        __disable_irq();
        buffer->size--;
        __set_PRIMASK(masking_state);
    }
    
    return data;   
}

