#include "runningLED.h"

void initLEDGPIO(void) {
	
	//enable port B
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
	
	//configure MUX settings to make B0, B1, B2 GPIO pins
	
	PORTB->PCR[1] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[1] |= PORT_PCR_MUX(1);
	
	PORTB->PCR[2] &= PORT_PCR_MUX_MASK;
	PORTB->PCR[2] |= PORT_PCR_MUX(1);
	
	PORTB->PCR[3] &= PORT_PCR_MUX_MASK;
	PORTB->PCR[3] |= PORT_PCR_MUX(1);
	
	//Set data direction to output
	
	PTB->PDDR |= (MASK(0) | MASK(1) | MASK(2));
}