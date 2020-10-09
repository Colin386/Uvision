#include "MKL25Z4.h"                    // Device header
#define MAX_SIZE 10

typedef struct {
	char buffer[MAX_SIZE];
	int start;
	int stop;
	int size;
} buffer;

void constructBuffer(buffer* newBuffer);
int isEmpty(buffer* b);
int isFull(buffer* b);
int enqueue(char d, buffer* b);
char dequeue(buffer* b);

