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
	uint32_t reset_LED = 0x00000008;
	uint32_t AtMax = 0x00000400;
	PTC->PDOR = reset_LED;
	
	for (;;) {
		uint32_t regState = PTC->PDOR;
		
		if ((regState) == AtMax) { //only if bits are at 7 will this cause it to be false
			PTC->PDOR = reset_LED; //add eight to avoid the first three bit
		} else {
			PTC->PDOR = PTC->PDOR << 1;
		}
		
		osDelay(500);
		
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
  initLEDGPIO();
 
  osKernelInitialize();                 // Initialize CMSIS-RTOS
  osThreadNew(app_main, NULL, NULL);    // Create application main thread
	osThreadNew(movingLED, NULL, NULL);
  osKernelStart();                      // Start thread execution
  for (;;) {}
}

