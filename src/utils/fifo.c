/**
 *
 * say something about project
 *
 * fifo.c
 *
 * v1.0
 *
 * say something about file
 */
#include "board.h"

#include "fifo.h"




void fifo_create(struct fifo_s *fifo, uint8_t *buf, uint16_t size)
{
    fifo->head = 0;
    fifo->tail = 0;
    fifo->data = buf;
    fifo->size = size;
}


int8_t fifo_write(struct fifo_s *fifo, uint8_t c)
{
    if(fifo->head + 1 == fifo->tail)
    {
        return -1;
    }
   
    fifo->data[fifo->head] = c;
    fifo->head++;
    fifo->cnt++;
    if(fifo->head >= fifo->size)
    {
        fifo->head = 0;
    }
    
    return 0;
}


void fifo_write_force(struct fifo_s *fifo, uint8_t c)
{  
    if(fifo->head + 1 == fifo->tail)
    {
        fifo->tail = fifo->head;
    }
    fifo->data[fifo->head] = c;
    fifo->head++;
    fifo->cnt++;
    if(fifo->head >= fifo->size)
    {
        fifo->head = 0;
        if(fifo->tail == 0)
        {
            fifo->tail = fifo->size -1;
        }
    } 
}


int8_t fifo_read(struct fifo_s *fifo, uint8_t* c)
{
    if(fifo->head == fifo->tail)
    {
        return -1;
    }

    *c = fifo->data[fifo->tail];
    fifo->tail++;
    fifo->cnt--;
    if(fifo->tail >= fifo->size)
    {
        fifo->tail = 0;
    }

    return 0;
}


bool fifo_is_empty(struct fifo_s *fifo)
{
    return (fifo->head == fifo->tail);
}

uint16_t fifo_get_count(struct fifo_s *fifo)
{
	return fifo->cnt;
}

uint16_t fifo_get_tail_index(struct fifo_s *fifo)
{
    return fifo->tail;
}


void fifo_set_tail_index(struct fifo_s *fifo, uint16_t new_index)
{
    fifo->tail = new_index;
}


uint8_t* fifo_get_tail(struct fifo_s *fifo)
{
    return fifo->data+fifo->tail;
}


#define IS_TAIL_BEHAND_HEAD (fifo->tail < fifo->head)
#define IS_TAIL_FRONT_HEAD  (fifo->tail > fifo->head)
#define IS_BEYOND_HEAD(x)   (((x)>fifo->head && (x)<fifo->tail && IS_TAIL_FRONT_HEAD) \
                          || ((x)>fifo->head && (x)>fifo->tail && IS_TAIL_BEHAND_HEAD))

void fifo_set_tail(struct fifo_s *fifo, uint8_t* new_tail)
{
    uint16_t new_index;

    new_index = new_tail - fifo->data;
    if(new_index > fifo->size)
    {
        new_index -= (fifo->size-1);
    }

    if(IS_TAIL_BEHAND_HEAD)
    {
        if(new_index > fifo->head)
        {
            fifo->tail = fifo->head;
        }
        else
        {
            fifo->tail = new_index;
        }
    }
    else if(IS_TAIL_FRONT_HEAD)
    {
        if(IS_BEYOND_HEAD(new_index))
        {
            fifo->tail = fifo->head;
        }
        else
        {
            fifo->tail = new_index;
        }
    }
    else //end to end
    {
        fifo->tail = fifo->head;
    }
}
