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

osThreadId_t redLED_Id, greenLED_Id, blueLED_Id, control_Id;
osEventFlagsId_t led_flag;

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
	led_flag = osEventFlagsNew(NULL);
  for (;;) {}
}

void red_LED (void *argument) {
	for (;;) {
		
		/*controlling with thread flags
		osThreadFlagsWait(0x00000001, osFlagsWaitAny, osWaitForever);
		LightLED(RED);
		osDelay(1000);
		OffLED(RED);
		osDelay(1000);*/
		
		osEventFlagsWait(led_flag, 0x0001, osFlagsWaitAll, osWaitForever);
		osEventFlagsClear(led_flag, 0x0001);
		LightLED(RED);
		osDelay(1000);
		OffLED(RED);
		osDelay(1000);
		
	}
	
}

void green_LED (void *argument) {
	for (;;) {
		
		/*controlling with thread flags
		osThreadFlagsWait(0x00000001, osFlagsWaitAny, osWaitForever);
		LightLED(GREEN);
		osDelay(1000);
		OffLED(GREEN);
		osDelay(1000);*/
		
		osEventFlagsWait(led_flag, 0x0002, osFlagsWaitAll, osWaitForever);
		osEventFlagsClear(led_flag, 0x0002);
		LightLED(GREEN);
		osDelay(1000);
		OffLED(GREEN);
		osDelay(1000);
	}
	
}

void blue_LED (void *argument) {
	for (;;) {
		
		/*controlling with thread flags
		osThreadFlagsWait(0x00000001, osFlagsWaitAny, osWaitForever);
		LightLED(BLUE);
		osDelay(1000);
		OffLED(BLUE);
		osDelay(1000);*/
		osEventFlagsWait(led_flag, 0x0004, osFlagsWaitAll, osWaitForever);
		osEventFlagsClear(led_flag, 0x0004);
		LightLED(BLUE);
		osDelay(1000);
		OffLED(BLUE);
		osDelay(1000);
	}
}

void control_thread (void *argument) {
	for(;;) {
		
		//controlling with thread flags
		/*osThreadFlagsSet(redLED_Id, 0x0000001);
		osDelay(1000);
		osThreadFlagsSet(greenLED_Id, 0x0000001);
		osDelay(1000);
		osThreadFlagsSet(blueLED_Id, 0x0000001);
		osDelay(1000);*/ 
		
		osEventFlagsSet(led_flag, 0x00000001);
		osDelay(1000);
		osEventFlagsSet(led_flag, 0x00000002);
		osDelay(1000);
		osEventFlagsSet(led_flag, 0x00000004);
		osDelay(1000);
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
	led_flag = osEventFlagsNew(NULL);
	
	
  redLED_Id = osThreadNew(red_LED, NULL, NULL);    // Create application main thread
	greenLED_Id = osThreadNew(green_LED, NULL, NULL);
	blueLED_Id = osThreadNew(blue_LED, NULL, NULL);
	control_Id = osThreadNew(control_thread, NULL, NULL);
	//osThreadNew(blue_LED, NULL, NULL);
  osKernelStart();                      // Start thread execution
  for (;;) {}
}
