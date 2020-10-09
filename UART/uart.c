#include "MKL25Z4.h"                    // Device header
#include "circular.h"

#define BAUD_RATE 9600
#define UART_TX_PORTE22 22
#define UART_RX_PORTE23 23
#define UART2_INT_PRIO 128
#define RED_LED 18 //PortB Pin 18
#define GREEN_LED 19 //Port B Pin 19
#define BLUE_LED 1 //PortD Pin 1
#define MASK(x) (1 << (x))

buffer circular_buff_RX;
buffer* buff_RX = &circular_buff_RX;

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
	
	//construct the circular buffer
	constructBuffer(buff_RX);
	
	NVIC_SetPriority(UART2_IRQn, 1);
	NVIC_ClearPendingIRQ(UART2_IRQn);
	NVIC_EnableIRQ(UART2_IRQn);
	
	
}

/*UART2 Transmit Poll*/
void UART2_Transmit_Poll(uint8_t data)
{
	while (!(UART2->S1 & UART_S1_TDRE_MASK));
	UART2->D = data;
}


/*UART2 Receive Poll */
uint8_t UART2_Receive_Poll(buffer* b) 
{
	while(!(UART2->S1 & UART_S1_RDRF_MASK));
	return UART2->D;
}

/*UART Receive Interrupt*/
void UART2_IRQHandler(void) {
	NVIC_ClearPendingIRQ(UART2_IRQn);
	if (UART2->S1 & UART_S1_RDRF_MASK) {
		//Character receieved
		if (!isFull(buff_RX)) {
			enqueue(UART2->D, buff_RX);
		}
		else { //buffer is full, ignore the bit
			;
		}
		
	}
}

static void delay(volatile uint32_t nof) {
	while(nof!=0) {
		__asm("NOP");
		nof--;
	}
}

/**Lighting functions
	*
	*
	*/

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



/*main function*/
int main(void)
{
	uint8_t rx_data = 0x69;
	
	
	
	SystemCoreClockUpdate();
	InitGPIO();
	initUART2(BAUD_RATE);
	OffLED(RED);
	OffLED(BLUE);
	OffLED(GREEN);
	while(1)
	{
		uint8_t info;
		/*RX and TX*/
		//UART2_Transmit_Poll(0x69);
		info = dequeue(buff_RX);
		
		
		switch (info) { //program does nothing if queue is empty. Will switch colour when detect signal.
			case 6:
				LightLED(RED);
				break;
			case 4:
				OffLED(RED);
				break;
			case 10:
				LightLED(GREEN);
				break;
			case 8:
				OffLED(GREEN);
				break;
			case 18:
				LightLED(BLUE);
				break;
			case 16:
				OffLED(BLUE);
				break;
			default:
				break;
			
		}
		
		//delay(0x80000);
	}
}



