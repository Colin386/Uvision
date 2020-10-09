#include "MKL25Z4.h"                    // Device header
#define RED_LED 18 //PortB Pin 18
#define GREEN_LED 19 //Port B Pin 19
#define BLUE_LED 1 //PortD Pin 1
#define MASK(x) (1 << (x))
#define FREQ 20971


enum colour
{
	RED, BLUE, GREEN
};

void InitGPIO(void)
{
	//Enable Clock to PORTB and PORTD
	SIM->SCGC5 |= ((SIM_SCGC5_PORTB_MASK) | (SIM_SCGC5_PORTD_MASK)); 
	
	//Configure MUX setings to make all 3 pins GPIO
	PORTB->PCR[RED_LED] &= ~PORT_PCR_MUX_MASK; //clears all the register
	PORTB->PCR[RED_LED] |= PORT_PCR_MUX(1); //set the MUX to allow the pins to act as GPIO
	
	PORTB->PCR[GREEN_LED] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[GREEN_LED] |= PORT_PCR_MUX(1);
	
	PORTD->PCR[BLUE_LED] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[BLUE_LED] |= PORT_PCR_MUX(1);
	
	//Set Data Direction Registers for PortB and PortD
	PTB->PDDR |= (MASK(RED_LED) | MASK(GREEN_LED));
	PTD->PDDR |= MASK(BLUE_LED);
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

