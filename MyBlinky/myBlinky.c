#include "MKL25Z4.h"                    // Device header
unsigned int counter = 0;


enum colour
{
	RED, BLUE, GREEN
};

/* Lighting function */
void InitGPIO(void);
void LightLED(enum colour c);
void OffLED(int colour);
void delay(unsigned long time);
void light_seq(unsigned int timeDelay);
void LightLEDTime(enum colour c, unsigned int timems);



/* Main function */
int main(void)
{
	
	SystemCoreClockUpdate();
	InitGPIO();
	while(1)
	{
		counter++;
		if(counter > 0x0F)
			counter = 0;
		light_seq(100);
	}
	
	
}

void light_seq(unsigned int timeDelay)
{
	LightLEDTime(RED, timeDelay);
	LightLEDTime(GREEN, timeDelay);
	LightLEDTime(BLUE, timeDelay);
	
	
}
