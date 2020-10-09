/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"

#include "MKL25Z4.h"                    // Device header
#define RED_LED 18 //PortB Pin 18
#define GREEN_LED 19 //Port B Pin 19
#define BLUE_LED 1 //PortD Pin 1
#define MASK(x) (1 << (x))
#define FREQ 20971

osMutexId_t myMutex;

volatile int LEDFlag = 0;

static void delay(volatile uint32_t nof) {
  while(nof!=0) {
    __asm("NOP");
    nof--;
  }
}

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
			
			PTB->PDOR &= ~MASK(RED_LED);
			break;
		case BLUE:
			
			PTD->PDOR &= ~MASK(BLUE_LED);
			break;
		case GREEN:
			
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
 
/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/
void app_main (void *argument) {
 
  // ...
  for (;;) {}
}

void red_LED (void *argument) {
	for (;;) {
		osMutexAcquire(myMutex, osWaitForever);
		LightLED(RED);
		osDelay(1000);
		OffLED(RED);
		osDelay(1000);
		osMutexRelease(myMutex);
	}
	
}

void green_LED (void *argument) {
	for (;;) {
		osMutexAcquire(myMutex, osWaitForever);
		LightLED(GREEN);
		osDelay(1000);
		OffLED(GREEN);
		osDelay(1000);
		//osMutexRelease(myMutex);
	}
	
}

const osThreadAttr_t thread_attr = {
	.priority = osPriorityNormal1
};



int main (void) {
 
  // System Initialization
  SystemCoreClockUpdate();
	InitGPIO();
	OffLED(RED);
	OffLED(GREEN);
	OffLED(BLUE);
	
  // ...
 
  osKernelInitialize();                 // Initialize CMSIS-RTOS
	myMutex = osMutexNew(NULL);
  osThreadNew(red_LED, NULL, NULL);    // Create application main thread
	osThreadNew(green_LED, NULL, NULL);
  osKernelStart();                      // Start thread execution
  for (;;) {}
}
