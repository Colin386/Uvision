/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
#include "MKL25Z4.h"
#include "runningLED.h"

#define SW 6


void initSwitch() {
	
	//Turn on the port
	SIM->SCGC5 |= (SIM_SCGC5_PORTD_MASK);
	
	//Set up the pin control register
	PORTD->PCR[SW] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[SW] |= PORT_PCR_MUX(1);
	
	//Enable pullup resistors
	PORTD->PCR[SW] |= (MASK(1) | MASK(0));
	PORTD->PCR[SW] |= PORT_PCR_IRQC(10);
	
	//Set portD switch to input
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
	
	//put code to simulate change of state of motor
	
}

void movingLED (void *argument) {
	
	//only set to zero bits in pos 1, 2, 3
	uint32_t reset_LED = 0xFFFFFFF1;
	uint32_t notAtMax = 0x0000000E;
	PTB->PDOR &= reset_LED;
	
	for (;;) {
		uint32_t regState = PTB->PDOR;
		
		if ((~regState) & notAtMax) { //only if bits are at 7 will this cause it to be false
			PTB->PDOR += 2; //add two to avoid the first bit
		} else {
			PTB->PDOR &= reset_LED;
		}
		
		osDelay(1000);
	}
}
/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/
void app_main (void *argument) {
 
  // ...
  for (;;) {}
}
 
int main (void) {
 
  // System Initialization
  SystemCoreClockUpdate();
  // ...
 
  osKernelInitialize();                 // Initialize CMSIS-RTOS
  osThreadNew(app_main, NULL, NULL);    // Create application main thread
	osThreadNew(movingLED, NULL, NULL);
  osKernelStart();                      // Start thread execution
  for (;;) {}
}
