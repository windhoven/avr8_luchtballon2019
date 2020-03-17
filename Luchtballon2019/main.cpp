/*
 * luchtb.c
 *
 * Created: 31-12-2011 13:35:57
 *  Author: Sharon
 * 
 * # define F_CPU 1000000UL
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <avr/eeprom.h>

uint8_t EEMEM deviceConfig = { 
	0x03 // Address
	 };

#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)// 1mhz = 6, 8mhz = 51

#define BUFFER_SIZE 6

#define B_GEEL 1
#define B_ORANJE 3

#define RANDOMNESS 30

#define ALL_LED_PINS ((1<<PINB0) | (1<<PINB1) | (1<<PINB2) | (1<<PINB3))

// The inputted commands are never going to be
// more than 8 chars long.
// volatile so the ISR can alter them
volatile static unsigned char rx_buffer[BUFFER_SIZE];
static unsigned char  command_in[BUFFER_SIZE];
volatile static unsigned char data_count;
volatile static unsigned char command_ready;
bool endOfCommand = false;

#define TRUE 1
#define FALSE 0

volatile uint8_t ignore = TRUE;

volatile uint16_t lastCmdCount =0;

volatile uint8_t misschien =30;	

volatile uint8_t b1_geel_aan = FALSE;
volatile uint8_t b1_oranje_aan = FALSE;
volatile uint8_t b1_reset = 0;

volatile uint8_t b2_geel_aan = FALSE;
volatile uint8_t b2_oranje_aan = FALSE;
volatile uint8_t b2_reset = 0;

unsigned char eAddress = 0;

void USART_Init(void)
{
 /* Set baud rate */
   UBRRH = (BAUD_PRESCALE >> 8); // Load upper 8-bits of the baud rate value into the high byte of the UBRR register
   UBRRL = BAUD_PRESCALE; // Load lower 8-bits of the baud rate value into the low byte of the UBRR register
   
 /* Enable receiver and transmitter */
 UCSRB = (1<<RXEN)|(1<<RXCIE); //|(1<<TXEN);
 
 /* Set frame format: 8data, 1stop bit, no parity */
 UCSRC = (1 << UCSZ0) | (1 << UCSZ1); // Use 8-bit character sizes
}

// Reset to 0, ready to go again		
void resetBuffer(void) {
	//for( int i = 0; i < BUFFER_SIZE;  ++i ) {
		//rx_buffer[i] = 0;
	//}
	rx_buffer[0] = 0;
	data_count = 0;
}

void processCommand(void) {
	// process command
	command_ready = FALSE;
	uint8_t iValue = command_in[1];
	if (iValue >= 128) {
		misschien = RANDOMNESS; // branders ON		
	} else {
		misschien = 0; // disable random till lastCmdCount = 0;
		b1_geel_aan = FALSE;
		b2_geel_aan = FALSE;
		b1_oranje_aan = FALSE;
		b2_oranje_aan = FALSE;
		PORTB = ALL_LED_PINS;
	}
	
	ignore = TRUE;
}

void putCharToBuffer(unsigned char c) {
	if (command_ready == TRUE)  // commando still needs to be processed
		return;	
	
	if (data_count >= BUFFER_SIZE) {
		// too much data
		resetBuffer();
		endOfCommand = false;
		ignore = TRUE;
	}
	
	if (data_count == 0 && c == eAddress) {
		// right address
		ignore = FALSE;
	}

	rx_buffer[data_count++] = c;
	
	if (c == '\r') { // End of line!	
		if (endOfCommand == true) {	
			if (ignore == FALSE) {
				command_ready = TRUE;
				// copy command, so we can reset the buffer
				for( int i = 0; i < BUFFER_SIZE;  ++i ) {
					command_in[i] = rx_buffer[i];				
				}
				// process command
				processCommand();
			}
			ignore = TRUE;
			resetBuffer();
			lastCmdCount = 8192;
		}
	} else {
		endOfCommand = false;
	}
	if (c == '\n') { // End of line!
		endOfCommand = true;
	}
}

/*
 * ISR RX complete
 * Receives a char from UART and stores it in ring buffer.
 */
ISR(USART_RX_vect) {	
	// Get data from the USART in register
	unsigned char temp = UDR;
	putCharToBuffer(temp);	
}

uint16_t myRandomValue(uint8_t ibase, uint8_t irand) {
	return ibase +(rand() / (RAND_MAX / irand + 1));	
}

void brander(volatile uint8_t& b_geel_aan,volatile uint8_t& b_oranje_aan,volatile uint8_t& b_reset, char pin_geel /* 1 */, char pin_oranje /* 0 */) {
	if (b_geel_aan == TRUE) {
		if (myRandomValue(0,B_ORANJE) ==0) {
			PORTB &= ~(1 << pin_geel); // High
			b_oranje_aan = TRUE;
		} else {
			PORTB  |= (1 << pin_geel);  // Low
			b_oranje_aan = FALSE;
		}
	} else {
		if (b_oranje_aan == TRUE) {
			b_reset+=1;
			
			if (b_reset >=5)
			{
				PORTB |= (1 << pin_geel);  // Low
				b_oranje_aan = FALSE;
				
				b_reset = 0;
			}
		}
	}
	
	if (myRandomValue(0,B_GEEL) ==0) { 
		if (b_geel_aan == FALSE) {
			PORTB &= ~(1 << pin_oranje);
			b_geel_aan = TRUE;
		} else {
			if (b_oranje_aan == FALSE) {
				PORTB |= (1 << pin_oranje);
				b_geel_aan = FALSE;
			}
		}
	}
}

/*
void MyDelay(unsigned int t) {			
	while (t > 0) {		
		t--;				
	}	
}
*/

/*
  Read random seed from eeprom and write a new random one.
*/
void initrand()
{
        uint32_t state;
        static uint32_t EEMEM sstate = 1;

        state = eeprom_read_dword(&sstate);

        // Check if it's unwritten EEPROM (first time). Use something funny in that case.
        if (state == 0xffffffUL)
                state = 0xDEADBEEFUL;
        srand(state);
		
		state = !state;
        eeprom_write_dword(&sstate,rand());
		 
		 misschien = RANDOMNESS;
} 

int main(void)
{		
	//DDRB  = 0x0F; // set PORTB for output = 0xff
	//PORTB  = 0x0F; // 0x00 = OFF all LEDs initially
	DDRB = (1<<DDB0) | (1<<DDB1) | (1<<DDB2) | (1<<DDB3) ; // = outputs voor branders
	PORTB  = ALL_LED_PINS;
	
	USART_Init();
	
	eAddress = eeprom_read_byte(&deviceConfig);

	
	sei();

	initrand();
	
	uint16_t doNothingTime = 0;
	uint8_t tel_aan =0;
	while(1)
	{

		//TODO:: Please write your application code
		if (doNothingTime > 0) {
			doNothingTime--;
		} else {
			if (misschien ==  RANDOMNESS || tel_aan > 0 || ( b1_geel_aan > 0  || b2_geel_aan > 0 || b1_oranje_aan > 0 || b2_oranje_aan > 0) ) {
				
				if (tel_aan > 0) {
					tel_aan--;
				}
				if (misschien ==  RANDOMNESS && tel_aan ==0) {
					tel_aan =  myRandomValue(50,50);
				}
				
				uint8_t branders =	myRandomValue(0,4);
				
				if (branders == 1 || branders == 3) { // bit 0
					brander(b1_geel_aan,b1_oranje_aan, b1_reset, PINB1 , PINB0 );
				}
				if (branders == 2 || branders == 3) { // bit 1
					brander(b2_geel_aan,b2_oranje_aan, b2_reset, PINB2 , PINB3 );
				}
				if (branders >= 1 && branders <= 3) {
					doNothingTime = myRandomValue(100,200);
					} else {
					b1_geel_aan = FALSE;
					b2_geel_aan = FALSE;
					b1_oranje_aan = FALSE;
					b2_oranje_aan = FALSE;
					PORTB = ALL_LED_PINS;
					doNothingTime = myRandomValue(50,100);
				}
				} else {
				doNothingTime =2000;
			}
			
			
			if (lastCmdCount == 0) {
				misschien =  myRandomValue(0,254); //rand() / (RAND_MAX / 254 + 1) ;
			} else {
				--lastCmdCount;
			}
		}
	}
	
	return 1;
}

