#define F_CPU 16000000
#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#define USART_BAUDRATE 57600
#define UBRR_VALUE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)
#define ADCIndex 16

volatile uint16_t ADCvalue;
volatile uint32_t revTickAvg;
volatile double revTime;
volatile uint8_t revCtr;
volatile uint8_t T1Ovs2;
volatile uint8_t speed;
volatile int8_t error;
volatile uint8_t Flag;
volatile uint16_t revTickSig;
volatile uint16_t revTick[50];
volatile uint16_t ticks;
char dataConv[ADCIndex];
char buffer[6];
char outs[30];
uint8_t uartSpeed;

void UART_init( void )
{
	UBRR0H = (uint8_t)(UBRR_VALUE >> 8); //Enable baud rate to 57600
	UBRR0L = (uint8_t)UBRR_VALUE; //Enable baud rate to 57600
	UCSR0C = (1 << UCSZ01) |  (1 << UCSZ00); //Data is 8 bit size, and there is 1 stop bit
	UCSR0B = (1 << TXEN0) | (1 << RXEN0); //Enable transmitter and receiver
}


//This function transmits the data, each char is entered into UDR0 register and transmitted to the serial monitor
void UART_Transmit(char data){
	while (!(UCSR0A & (1 << UDRE0)));  // Cycles through each char until UDRE0 is 1
	UDR0 = data;
}

//Sends data to UART_Transmit
void USART_tx_string(char *data) {
	while (*data != '\0') { //Cycles through each char in data
		UART_Transmit(*data); //Sends
		data++;
	}
}

void ADC_init(void){
	DDRC |= 0; //Set PORTC as input
	ADMUX |= (1 << REFS0) | (0 << MUX0); //Sets Vref as Vcc, operates on channel 0
	ADCSRA |= (1 << ADEN) | (1 << ADATE) | (1 << ADPS2) | (1 << ADIE); //Enable interrupt, prescaler at 128, enable ADC and auto trigger
	ADCSRB |= (1 << ADTS2) | (1 << ADTS0); //Auto trigger with timer0 interrupt
}

void Timer1(void){
	DDRB |= (0 << DDB0);
	//Set Initial Timer value
	TCNT1 = 0;
	////First capture on rising edge
	TCCR1A = 0;
	TCCR1B |= (1<<CS11) | (1<<ICES1);
	TCCR1C = 0;
	// Interrupt setup
	// ICIE1: Input capture
	// TOIE1: Timer1 overflow
	TIFR1 = (1<<ICF1) | (1<<TOV1);// clear pending
	TIMSK1 = (1<<ICIE1) | (1<<TOIE1);// and enable
}

//Creates a 0.01 sec delay
void Timer3(void){
	TCCR3B |= (1 << CS31) | (1 << CS30) | (1 << WGM32); //Set Prescaler to 64 and enable CTC mode
	TCNT3 = 0; //Counter starts at 0
	OCR3A = 2499; //Counts up till 2499 (2500 counts)
	TIMSK3 = (1<<OCIE3A);
}


void Timer0(void){
	DDRD |= (1 << PD6); 
	TCNT0 = 0;  //Set timer0 count zero
	TCCR0A |= (1 << WGM00) | (1 << WGM01) | (1 << COM0A1); //Enables fast PWM, non inverting
	TCCR0B = (1 << CS01); //Set Fast PWM with Fosc/64 Timer0 clock
}

unsigned char UART_Receive(void){
	while(!(UCSR0A & (1 << RXC0))); //Stuck in an infinite loop until data is received in USCR0A
	return UDR0; //Returns data
}

//Receives User input
void UART_getLine(char* buf, uint8_t n){
	uint8_t bufIdx = 0; 
	char c;
	
	do{
		c = UART_Receive(); //Initializes C as the data received
		buf[bufIdx++] = c; //Iterates through each char in the data and stores it
	}
	while((bufIdx < n) && (c != '\r')); 
	
	buf[bufIdx] = 0;
}


int main(void)
{
	//Instantiate all functions
	UART_init();
	ADC_init();
	Timer0();
	Timer1();
	Timer3();
	DDRD |= (1 << 3) | (1 << 4); //PD3 and PD4 outputs
	PORTD |= (1 << PD3) | (1 << PD4); //Standby and A02 enabled (Direction of the motor)
	PORTD &= ~(1 << PD2); // A01 = 0
	sei(); // Enable global interrupts
	speed = 0; //Set default speed to 0
	char controlActive = 0; //Variable used to call OCR0A math functions, used when user input for speed is given

	while (1)
	{
		// Check for user input
		if (UCSR0A & (1 << RXC0)) { //When data is received, proceed
			UART_getLine(buffer, 5); //Stores user input in buffer
			uartSpeed = atoi(buffer); //ASCII to integer
			if (uartSpeed >= 20 && uartSpeed <= 90) { //Limits for speed
				speed = uartSpeed;
				controlActive = 1; //control variable is set
				} else {
				controlActive = 0; // stop printing and control
				OCR0A = 0;
			}
		}

		// Run control logic continuously when active
		if (controlActive) {
			OCR0A = 2.95 * speed; //Converts user input speed to PWM
			error = speed - revTime;
			if(abs(error) > 1){ //If error is greater than 1, adjust OCR0A
				OCR0A = 2.95 * speed + 0.72 * error;
			}

			if (Flag == 1){
				snprintf(outs, sizeof(outs), "%d,%.3f\n", speed, revTime); //Prints speed and revTime to the terminal
				USART_tx_string(outs);
				Flag = 0;
			}
		}
	}
}

// capture ISR
ISR(TIMER1_CAPT_vect) {
	ticks = ICR1;
	revTickSig = ticks + (T1Ovs2 * 0x10000L);
	revTick[revCtr++] = revTickSig;

	//Calculates averages ticks for every 20 captures
	if (revCtr == 20) {
		revTickAvg = 0;
		for (int i = 0; i < 20; i++) {
			revTickAvg += revTick[i];
		}
		revTickAvg /= 20.0;

		// Time per revolution in seconds
		revTime = (double)(60.0 * 1000000) / (144.0 * revTickAvg * 8 * 0.0625);
		// Send result via UART
		if(speed == 0){
			snprintf(outs, sizeof(outs), "%.2f \r\n", revTime);
			USART_tx_string(outs);
		}
		revCtr = 0;
		Flag = 1;
	}

	TCNT1 = 0;
	T1Ovs2 = 0;
}
// Overflow ISR
ISR(TIMER1_OVF_vect) {
	// increment overflow counter
	T1Ovs2++;
}

ISR(TIMER3_COMPA_vect) {
	ADCSRA |= (1 << ADSC); //Start ADC conversion
}

ISR(ADC_vect){
	ADCvalue = ADC; //Stores converted ADC value into a 16 bit variable
	if(speed == 0){ //If user input isn't given RPM is controlled by the potentiometer
		OCR0A = ADCvalue / 4; //Scales 0 - 1023 from potentiometer to 0 - 255
	}
}
