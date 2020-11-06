/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
#include "MKL25Z4.h"                    // Device header
#include "runningLED.h"
#include "notes.h"

#define MASK(x) (1 << (x))

#define BAUD_RATE 9600
#define UART_TX_PORTE22 22
#define UART_RX_PORTE23 23
#define UART2_INT_PRIO 128

#define PTD0_PIN 0
#define PTD1_PIN 1
#define PTD2_PIN 2
#define PTD3_PIN 3
#define PRESCALAR 7
#define PS_ACTUAL 128
#define DEFAULT_PWM_FREQ 50
#define TRUE 1
#define FALSE 0

#define AUDIO 0

#define POWER 80
#define LOWPOWER 20

int sheet1[] = {D6,C6,D6,E6,D6,A5,C6,C6,C6,A6,Gb6,D6,C6,D6,E6,D6,A5,B5,C6,D6,E6,D6,C6,G5,A5,C6,D6,E6,D6,A5,C6,G6,G6,A6,Gb6,D6,E6,F6,G6,A6,G6,F6,E6,C6,A5,A6,D7,E7,D7};
int count1[] = {3 ,1 ,1 ,1 ,2 ,2 ,2 ,2 ,2 ,2 ,6  ,3 ,1 ,1 ,1 ,2 ,2 ,2 ,2 ,2 ,2 ,3 ,2 ,2 ,2 ,1 ,1 ,1 ,2 ,2 ,2 ,4 ,1 ,1 ,4  ,1 ,1 ,2 ,2 ,2 ,1 ,1 ,2 ,2 ,3 ,1 ,1 ,1 ,10};
int length1 = sizeof(sheet1)/sizeof(int);
 
int sheet2[] = {Eb5,C6,Ab5,Bb5,C6,Bb5,Ab5,F5,Eb5,Eb5,C6,Ab5,Bb5,C6,Bb5,Ab5,F5,Ab5,Ab5,Bb5,C6,Bb5,Ab5,F5,Ab5,Ab5};
int count2[] = {8  ,6 ,2  ,2  ,2 ,2  ,2  ,4 ,4  ,8  ,6 ,2  ,2  ,2 ,2  ,2  ,4 ,4  ,14 ,2  ,2 ,2  ,2  ,4 ,4  ,20};
int length2 = sizeof(sheet2)/sizeof(int);
 
int sheet3[] = {F5,C6,B5,C6,F6,C6,B5,C6,F5,Db6,C6,Db6,F6,Db6,C6,Db6,F5,D6,Db6,D6,F6,D6,Db6,D6,F5,Db6,C6,Db6,F6,Db6,C6,Db6,F5,C6,Bb5,C6,F6,C6,Bb5,C6,F5,C6,Bb5,C6,F6,C6,Bb5,C6,F5,D6,Db6,D6,F6,D6,Db6,D6,F5,Db6,C6,Db6,F6,Db6,C6,Db6,F5,C6,Bb5,C6,F6,C6,Bb5,C6,F5,C6,Bb5,C6,F6,C6,Bb5,C6};
int length3 = sizeof(sheet3)/sizeof(int);

osMessageQueueId_t motorMsg, audioMsg;

uint8_t RxData;

osEventFlagsId_t led_flag;

enum Direction
{
	LEFT, RIGHT, FORWARD, BACKWARD, FORWARD_LEFT, FORWARD_RIGHT, BACKWARD_LEFT, BACKWARD_RIGHT
};

void initUART2(uint32_t baud_rate)
{
	uint32_t divisor, bus_clock;
	
	//ACTIVATES CLOCK TO UART2 AND PORT E
	SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	
	PORTE->PCR[UART_TX_PORTE22] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[UART_TX_PORTE22] |= PORT_PCR_MUX(4);
	
	PORTE->PCR[UART_RX_PORTE23] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[UART_RX_PORTE23] |= PORT_PCR_MUX(4);
	
	UART2->C2 &= ~((UART_C2_TE_MASK) | (UART_C2_RE_MASK));

	bus_clock = (DEFAULT_SYSTEM_CLOCK)/2;
	divisor = bus_clock / (baud_rate * 16);
	UART2->BDH = UART_BDH_SBR(divisor >> 8);
	UART2->BDL = UART_BDL_SBR(divisor);
	
	UART2->C1 = 0;
	UART2->S2 = 0;
	UART2->C3 = 0;

	//enable transmit, recieve and recieve interrupt enable mask
	UART2->C2 |= ((UART_C2_TE_MASK) | (UART_C2_RE_MASK) | (UART_C2_RIE_MASK));
	
	NVIC_SetPriority(UART2_IRQn, 1);
	NVIC_ClearPendingIRQ(UART2_IRQn);
	NVIC_EnableIRQ(UART2_IRQn);
}

/*UART Receive Interrupt*/
void UART2_IRQHandler(void) {
	NVIC_ClearPendingIRQ(UART2_IRQn);
	if (UART2->S1 & UART_S1_RDRF_MASK) {
		//Character receieved
		RxData = UART2->D;
	}
}


//pinD0 channel 0, D1 channel 1, D2 channel 2, D3 channel 3
/* intiPWM() */
void initPWM(void)
{
	SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;
	
	PORTD->PCR[PTD0_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PTD0_PIN] |= PORT_PCR_MUX(4);
	
	PORTD->PCR[PTD1_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PTD1_PIN] |= PORT_PCR_MUX(4);
	
	PORTD->PCR[PTD2_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PTD2_PIN] |= PORT_PCR_MUX(4);
	
	PORTD->PCR[PTD3_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PTD3_PIN] |= PORT_PCR_MUX(4);
	
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;
	
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);
	
	TPM0->MOD = (DEFAULT_SYSTEM_CLOCK/PS_ACTUAL) / DEFAULT_PWM_FREQ;
	
	//Timer  settings
	TPM0->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM0->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(PRESCALAR));
	TPM0->SC &= ~(TPM_SC_CPWMS_MASK);
	
	//timer channel settings for pwm mode activation
	TPM0_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) |(TPM_CnSC_MSB_MASK) |(TPM_CnSC_MSA_MASK));
	TPM0_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	TPM0_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) |(TPM_CnSC_MSB_MASK) |(TPM_CnSC_MSA_MASK));
	TPM0_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	TPM0_C2SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) |(TPM_CnSC_MSB_MASK) |(TPM_CnSC_MSA_MASK));
	TPM0_C2SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	TPM0_C3SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) |(TPM_CnSC_MSB_MASK) |(TPM_CnSC_MSA_MASK));
	TPM0_C3SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	
}

void initAudio(void)
{
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	
	PORTB->PCR[AUDIO] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[AUDIO] |= PORT_PCR_MUX(3);
	
	SIM_SCGC6 |= SIM_SCGC6_TPM1_MASK;
	
	//TPM clock source select - MCGFLLCLK
	SIM_SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM_SOPT2 |= SIM_SOPT2_TPMSRC(1);
	
	TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM1->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7)); //clock mode and prescalar
	TPM1->SC &= ~TPM_SC_CPWMS_MASK; //center-aligned PWM select
	
	//mode, edge, and level selection - edge-aligned PWM High true pulse
	TPM1_C0SC &= ~((TPM_CnSC_ELSA_MASK) | (TPM_CnSC_ELSB_MASK) | (TPM_CnSC_MSA_MASK) | (TPM_CnSC_MSB_MASK));
	TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
}

void setDutyCycle(float percent, int channel)
{
	float multiple = percent/100.0;
	
	switch(channel) {
	case 0:
		TPM0_C0V = (unsigned int)(TPM0->MOD * multiple);
	  break;
	case 1:
		TPM0_C1V = (unsigned int)(TPM0->MOD * multiple);
	  break;
	case 2:
		TPM0_C2V = (unsigned int)(TPM0->MOD * multiple);
	  break;
	case 3:
		TPM0_C3V = (unsigned int)(TPM0->MOD * multiple);
	  break;
	}
	
}

void setFreq(unsigned int value, int channel) 
{
	
	float step_size;
	unsigned int steps_needed;
	step_size = ((float)PS_ACTUAL/(float)DEFAULT_SYSTEM_CLOCK);
	steps_needed = (unsigned int)((1/(float)value)/step_size);
	
	TPM0->MOD = steps_needed;
	
	//after you set the new frequency, the COV value needs to change
	setDutyCycle(50.0, channel);
}

void leftMotorControl(float power, int direction) {
	if(direction) {
		setDutyCycle(power, PTD0_PIN);
		setDutyCycle(0, PTD1_PIN);
		
	} else {
		setDutyCycle(0, PTD0_PIN);
		setDutyCycle(power, PTD1_PIN);
	}
}

void rightMotorControl(float power, int direction) {
	if(direction) {
		setDutyCycle(power, PTD2_PIN);
		setDutyCycle(0, PTD3_PIN);
		
	} else {
		setDutyCycle(0, PTD2_PIN);
		setDutyCycle(power, PTD3_PIN);
	}
}

void stop() {
	osEventFlagsClear(led_flag, 0x00000002);
	osEventFlagsSet(led_flag, 0x00000001);
	leftMotorControl(0, 0);
	rightMotorControl(0, 0);
}

void move(enum Direction direction) {
	
	osEventFlagsClear(led_flag, 0x00000001);
	osEventFlagsSet(led_flag, 0x00000002);
	switch(direction) {
	case LEFT:
		leftMotorControl(POWER, 1); // make left move backwards
		rightMotorControl(POWER, 0); // make right move forwards
		break;
	case RIGHT:
		leftMotorControl(POWER, 0); // make left move forwards
		rightMotorControl(POWER, 1); // make right move backwards
		break;
	case FORWARD:
		leftMotorControl(POWER, 0); // make left move forwards
		rightMotorControl(POWER, 0); // make right move forwards
		break;
	case BACKWARD:
		leftMotorControl(POWER, 1); // make left move backwards
		rightMotorControl(POWER, 1); // make right move backwards
		break;
	case FORWARD_LEFT:
		leftMotorControl(LOWPOWER, 0); // make left move forwards
		rightMotorControl(POWER, 0); // make right move forwards slightly faster
		break;
	case FORWARD_RIGHT:
		leftMotorControl(POWER, 0); // make left move forwards slightly faster
		rightMotorControl(LOWPOWER, 0); // make right move forwards 
		break;
	case BACKWARD_RIGHT:
		leftMotorControl(POWER, 1); // make left move backwards slightly faster
		rightMotorControl(LOWPOWER, 1); // make right move backwards 
		break;
	case BACKWARD_LEFT:
		leftMotorControl(LOWPOWER, 1); // make left move backwards 
		rightMotorControl(POWER, 1); // make right move backwards slightly faster
		break;
  }
}

void playNote(int beat, int freq, int count)
{
	TPM1->MOD = CLOCK_STEP/freq;
	TPM1_C0V = CLOCK_STEP/(2*freq);
	osDelay(beat);
	TPM1_C0V = 0;
	osDelay(beat*(count-1) + 1);
}
 
/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/

void tMotorControl (void *argument) {
	uint8_t dir;
	for (;;) {
		osMessageQueueGet(motorMsg, &dir, NULL, 0);
		switch(dir) {
		case 1:
			move(FORWARD);
			break;
		case 2:
			move(BACKWARD);
			break;
		case 3:
			move(LEFT);
			break;
		case 4:
			move(RIGHT);
			break;
		case 6:
			move(FORWARD_LEFT);
			break;
		case 7:
			move(FORWARD_RIGHT);
			break;
		case 8:
			move(BACKWARD_LEFT);
			break;
		case 9:
			move(BACKWARD_RIGHT);
			break;
		case 10:
			osEventFlagsSet(led_flag, 0x00000004);
			osEventFlagsClear(led_flag, 0x00000003);
			break;
		default:
			stop();
		}
		//osDelay(100);
		//RxData = 0;
	}
}

void tAudio(void *argument)
{
	//play startup tune
	playNote(200, D5, 2);
	playNote(200, D5, 2);
	playNote(200, Db5, 2);
	playNote(200, Db5, 2);
	playNote(200, E5, 2);
	playNote(200, E5, 2);
	playNote(200, Eb5, 2);
	playNote(200, Eb5, 2);
	osDelay(500);
	int tune = 1;
	int i = 0;
	int j = 0;
	int k = 0;
	for(;;)
	{
		osMessageQueueGet(audioMsg, &tune, NULL, 0);		
		switch(tune)
		{
		case 1:
			playNote(250, sheet1[i], count1[i]);
			i = (i >= length1-1) ? 0 : i+1;
			j = 0;
			k = 0;
			break;
		case 2:
			playNote(125, sheet2[j], count2[j]);
			j = (j >= length2-1) ? 0 : j+1;
			i = 0;
			k = 0;
			break;
		case 3:
			playNote(100, sheet3[k], 1);
			k = (k >= length3-1) ? 0 : k+1;
			i = 0;
			j = 0;
			break;
		}
	}
}

void tBrain(void *argument) {
	int tune = 1;
	uint8_t data;
	for(;;) {
		if (data != RxData) {
			data = RxData;
			if (data != 5) {
				osMessageQueuePut(motorMsg, &data, NULL, 0);
			} else {
				tune = (tune == 3) ? 1 : tune+1;
				osMessageQueuePut(audioMsg, &tune, NULL, 0);
			}
		}
	}
}

void movingLED (void *argument) {
	
	//only set to zero bits in pos 1, 2, 3
	uint32_t reset_LED = 0x00000008;
	uint32_t LED_reg_to_zero = 0xFFFFF807;
	uint32_t AtMax = 0x00000400;
	uint32_t AllOn = 0x000007F8;
	uint32_t back_LED = 0x00000800;
	
	PTC->PDOR = reset_LED;
	uint32_t amount_to_increment = 0;
	
	
	for (;;) {
		
		uint32_t status = osEventFlagsGet(led_flag);
		
		if (status == 0x00000002) { //moving running led
			uint32_t regState = PTC->PDOR;
		
			if (regState &= AtMax) { //triggers when the 10 bit is positive. This scene occurs when counter count all the way up or if all led are on
				
				PTC->PDOR &= LED_reg_to_zero;
				PTC->PDOR |= reset_LED;
				
			} else {
				
				amount_to_increment = PTC->PDOR & (~LED_reg_to_zero); //get only the bits responsible for the front led register
				PTC->PDOR += amount_to_increment;
			}
			
			//blink the front led at a fixed timing
			PTC->PDOR ^= back_LED;
			
		
		osDelay(500);
		}
		
		else if (status==0x00000001){ //stationery LED pattern
			PTC->PDOR |= AllOn;
			PTC->PDOR ^= back_LED;
			
			osDelay(250);
			
			
		} else {
			PTC->PDOR |= AllOn;
			osDelay(250);
			PTC->PDOR &= (~AllOn);
			osDelay(250);
			PTC->PDOR |= AllOn;
			osDelay(250);
			PTC->PDOR &= (~AllOn);
			osDelay(250);
			PTC->PDOR |= AllOn;
			osEventFlagsClear(led_flag, 0x00000004);
			osEventFlagsSet(led_flag, 0x00000001);
			RxData = 0;
		}
		
		
	}
}



int main (void) {
		// System Initialization
		initPWM();
	  initAudio();
		initLEDGPIO();
		initUART2(BAUD_RATE);
	
		osKernelInitialize();                 // Initialize CMSIS-RTOS
		motorMsg = osMessageQueueNew(10, sizeof(uint8_t), NULL);
	  audioMsg = osMessageQueueNew(10, sizeof(int), NULL);
		osThreadNew(tMotorControl, NULL,NULL);
	  osThreadNew(tAudio, NULL, NULL);
		osThreadNew(tBrain, NULL, NULL);
		osThreadNew(movingLED, NULL, NULL);
		led_flag = osEventFlagsNew(NULL);
		osKernelStart();                      // Start thread execution
		for (;;) {}
}
