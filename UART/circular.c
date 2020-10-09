#include "circular.h"

void constructBuffer(buffer* newBuffer) 
{
	
	newBuffer->start = 0;
	newBuffer->stop = 0;
	newBuffer->size = 0;
	
}

int isEmpty(buffer* b) 
{
	return (int)(b->size == 0);
}

int isFull(buffer* b)
{
	return (int)(b->size == MAX_SIZE);
}

int enqueue(char d, buffer* b)
{
	int wheretorecord;
	if(isFull(b))
	{
		return -1; //error, cannot enqueue any further
	}
	else
	{
		wheretorecord = b->start;
		b->buffer[wheretorecord] = d;
		b->size++;
		b->start = (b->start == MAX_SIZE -1) ? 0 : (b->start + 1);
		return 0;
	}
}

char dequeue(buffer* b) 
{
	int wheretoremove;
	char d;
	if(isEmpty(b))
	{
		return '!'; //error, cannot dequeue any further
	}
	else
	{
		wheretoremove = b->stop;
		d = b->buffer[wheretoremove];
		b->stop = (b->stop == MAX_SIZE-1) ? 0 : (b->stop + 1);
		b->size--;
		return d;
	}
}
