#include "MKL25Z4.h"                    // Device header
#include "music.h"


#define PTB0_PIN 0
#define PTB1_PIN 1
#define PRESCALAR 7
#define PS_ACTUAL 128

/* intiPWM() */

void initPWM(void)
{
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	
	PORTB->PCR[PTB0_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB0_PIN] |= PORT_PCR_MUX(3);
	
	PORTB->PCR[PTB1_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB1_PIN] |= PORT_PCR_MUX(3);
	
	SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;
	
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);
	
	TPM1->MOD = (DEFAULT_SYSTEM_CLOCK/PS_ACTUAL) / 50;
	
	TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM1->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(PRESCALAR));
	TPM1->SC &= ~(TPM_SC_CPWMS_MASK);
	
	TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) |(TPM_CnSC_MSB_MASK) |(TPM_CnSC_MSA_MASK));
	TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	
}

void setDutyCycle(float percent)
{
	float multiple = percent/100.0;
	
	TPM1_C0V = (unsigned int)(TPM1->MOD * multiple);
}

void setFreq(unsigned int value) 
{
	
	float step_size;
	unsigned int steps_needed;
	step_size = ((float)PS_ACTUAL/(float)DEFAULT_SYSTEM_CLOCK);
	steps_needed = (unsigned int)((1/(float)value)/step_size);
	
	TPM1->MOD = steps_needed;
	
	//after you set the new frequency, the COV value needs to change
	setDutyCycle(50.0);
}

static void delay(volatile uint32_t nof) {
  while(nof!=0) {
    __asm("NOP");
    nof--;
  }
}

static void duration(int fraction) {
	
	uint32_t base = 0x1000000;
	for (int i = fraction; i >= 1; i = i/2)
	{
		base = base >> 1;
	}
	
	delay(base);
	

}

static void play(int* m, int size)
{
	int i;
	int freq;
	int length;
	for (int i = 0; i < size; i = i+2)
	{
		freq = *m;
		m++;
		length = *m;
		m++;
		
		setFreq(freq);
		duration(length);
	}
	
	
}
int main()
{
	initPWM();
	while(1){
		int doh[] = {C5, 4, D5, 8, E5, 4, C5, 8, E5, 4, C5, 4, E5, 2};
		play(doh, 14);
		int re[] = {D5, 4, E5, 8, F5, 8, F5, 8, E5, 8, D5, 8, F5, 1};
		play(re, 14);
		//int mee[] = {E5, 4, F5, 8, G5, 4, E5, 8, G5, 4, E5, 4, G5, 2};
		//play(mee, 14);
		
}
	
	
}
