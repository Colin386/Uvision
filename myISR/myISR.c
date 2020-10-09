#include "MKL25Z4.h"
#define MASK(x) (1<<(x))
#define RED_LED 18
#define GREEN_LED 19
#define BLUE_LED 1
#define SW 6
#define FREQ 48000

volatile int color_num = 0;
enum colour
{
	RED, BLUE, GREEN
};

void initLED(void);
void initSwitch(void);
void LightLEDTime(enum colour c, unsigned int timems);
void delay(unsigned long time);
void OffLED(int colour);
void LightLED(enum colour c);





int main(void)
{
	SystemCoreClockUpdate();
	initSwitch();
	initLED();
	
	while(1)
	{
		LightLED((enum colour)color_num);
	};
	
	
	
}

void initLED(void)
{
	//Turn on the ports
	
	SIM->SCGC5 |= ((SIM_SCGC5_PORTB_MASK) | (SIM_SCGC5_PORTD_MASK)) ;
	
	//setting pin control registers (PCR)
	
	PORTB->PCR[RED_LED] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[RED_LED] |= PORT_PCR_MUX(1);
	
	PORTB->PCR[GREEN_LED] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[GREEN_LED] |= PORT_PCR_MUX(1);
	
	PORTD->PCR[BLUE_LED] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[BLUE_LED] |= PORT_PCR_MUX(1);
	

	
	// set pin data direction
	PTB->PDDR |= (MASK(RED_LED) | MASK(GREEN_LED));
	PTD->PDDR |= MASK(BLUE_LED);
	
}

void initSwitch()
{
	//Turn on the port
	SIM->SCGC5 |= (SIM_SCGC5_PORTD_MASK) ;
	
	//set up the pin control register
	PORTD->PCR[SW] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[SW] |= PORT_PCR_MUX(1);
	
	//ENABLE PULL UP RESISTORS
	PORTD->PCR[SW] |= (MASK(1) | MASK(0));
	PORTD->PCR[SW] |= PORT_PCR_IRQC(10); //falling edge
	
	//set portD switch to input
	PTD->PDDR &= ~(MASK(SW));
	
	
	//enable interrupts
	NVIC_SetPriority(PORTD_IRQn, 2);
	NVIC_ClearPendingIRQ(PORTD_IRQn);
	NVIC_EnableIRQ(PORTD_IRQn);
	
	
}

void PORTD_IRQHandler()
{
	//clear pending IRQ
	NVIC_ClearPendingIRQ(PORTD_IRQn);
	
	//UPDATING VARIABLE
	color_num = (color_num == 2) ? 0 : color_num+1;
	//Clear INT Flag
	PORTD_ISFR |= MASK(SW);
	
}





void LightLED(enum colour c)
{
	//LED is active LOW
	switch (c) {
		case RED:
			PTB->PDOR |= MASK(GREEN_LED);
			PTD->PDOR |= MASK(BLUE_LED);
			PTB->PDOR &= ~MASK(RED_LED);
			break;
		case BLUE:
			PTB->PDOR |= MASK(RED_LED);
			PTB->PDOR |= MASK(GREEN_LED);
			PTD->PDOR &= ~MASK(BLUE_LED);
			break;
		case GREEN:
			PTB->PDOR |= MASK(RED_LED);
			PTD->PDOR |= MASK(BLUE_LED);
			PTB->PDOR &= ~MASK(GREEN_LED);
			break;
		default:
			break;
	
	}
}

void OffLED(int colour)
{
	//LED is active LOW
	switch (colour) {
		case RED:
			PTB->PDOR |= MASK(RED_LED);
			break;
		case BLUE:
			PTD->PDOR |= MASK(BLUE_LED);
			break;
		case GREEN:
			PTB->PDOR |= MASK(GREEN_LED);
			break;
		default:
			break;
	}
}

void delay(unsigned long time)
{
	unsigned long count;
	for (count = 0; count < time; count++)
	{
		;
	}
	
}

void LightLEDTime(enum colour c, unsigned int timems)
{
	unsigned long ticks;
	ticks = FREQ * timems;
	LightLED(c);
	delay(ticks);
	OffLED(c);
	
	
}

