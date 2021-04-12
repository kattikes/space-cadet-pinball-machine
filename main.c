 /*
 * PinballMachine.c
 *
 * Created: 3/30/2021 1:24:14 PM
 * Author : CETSLoaner
 */ 

#define F_CPU 16000000UL
#define BAUD_RATE 9600
#define BAUD_PRESCALER (((F_CPU / (BAUD_RATE * 16UL))) - 1)

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdio.h>
#include "LCD.h"

volatile int gameHasBegun = 0;
volatile int readyToUpdateScore = 0;
volatile int score = 0;

volatile int sensorState1 = 0;
volatile int lastState1 = 0;
volatile int sensorState2 = 0;
volatile int lastState2 = 0;

typedef struct {
	int xy[10];
	} INT_ARRAY;

char str[16];

void UART_init(void) {
	//Set baud rate
	UBRR0H = (unsigned char) (BAUD_PRESCALER>>8);
	UBRR0L = (unsigned char) BAUD_PRESCALER;
	//Enable receiver/transmitter
	UCSR0B = (1<<RXEN0) | (1<<TXEN0);
	//2 stop bits, 8 data bits
	UCSR0C = (1<<UCSZ01) | (1<<UCSZ00);
	UCSR0C |= (1<<USBS0);
}

void UART_send(unsigned char data) {
	while(!(UCSR0A & (1<<UDRE0)));
	UDR0 = data;
}

void UART_putstring(char* StringPtr) {
	while(*StringPtr != 0x00) {
		UART_send(*StringPtr);
		StringPtr++;
	}
}

void Initialize() {
	//Set PD3 as input pin for break beam sensor 1
	DDRD &= ~(1<<DDD3);
	PORTD |= (1<<PORTD3);
	//Set PD2 as input pin for break beam sensor 2
	DDRD &= ~(1<<DDD2);
	PORTD |= (1<<PORTD2);
	
	//Clear power reduction for ADC
	PRR &= ~(1<<PRADC);
	//Select Vref = AVcc
	ADMUX |= (1<<REFS0);
	ADMUX &= ~(1<<REFS1);
	//Prescale ADC Clock by 1/128
	ADCSRA |= (1<<ADPS0);
	ADCSRA |= (1<<ADPS1);
	ADCSRA |= (1<<ADPS2);
	//Select channel 0
	ADMUX &= ~(1<<MUX0);
	ADMUX &= ~(1<<MUX1);
	ADMUX &= ~(1<<MUX2);
	ADMUX &= ~(1<<MUX3);
	//Autotriggering of ADC
	ADCSRA |= (1<<ADATE);
	//Free-running mode
	ADCSRB &= ~(1<<ADTS0);
	ADCSRB &= ~(1<<ADTS1);
	ADCSRB &= ~(1<<ADTS2);
	//Disable digital input buffer on ADC pin
	DIDR0 |= (1<<ADC0D);
	//Enable ADC
	ADCSRA |= (1<<ADEN);
	//Start conversation
	ADCSRA |= (1<<ADSC);
}

void startGameTimer(void) {
	//Disable global interrupts
	cli();
	
	//Prescale Timer 1 by 1/1024
	TCCR1B |= (1<<CS10);
	TCCR1B &= ~(1<<CS11);
	TCCR1B |= (1<<CS12);
	//Set Timer 1 to normal
	TCCR1A &= ~(1<<WGM10);
	TCCR1A &= ~(1<<WGM11);
	TCCR1B &= ~(1<<WGM12);
	TCCR1B &= ~(1<<WGM13);
	//Enable interrupt for Timer 1
	TIMSK1 |= (1<<OCIE1A);
	
	//Enable global interrupts
	sei();
}

/* Gets position of either Joystick0 or Joystick1. 
   Position is of the form (x, y), where 1 <= x, y <= 1023. 
   Top rightmost position is (1023, 1023). Bottom leftmost position is (1, 1).
   Centered position is about  (516, 517).
*/
INT_ARRAY getJoyStickPosition(int joyStickName) {
	int x = 0;
	int y = 0;
	INT_ARRAY position;
	if (joyStickName == 0) {
		//Switch to reading ADC0
		_delay_ms(10);
		ADMUX &= ~(1<<MUX0);
		ADMUX &= ~(1<<MUX1);
		//Save Xout
		x = ADC;
		//Switch to reading ADC1
		_delay_ms(10);
		ADMUX |= (1<<MUX0);
		ADMUX &= ~(1<<MUX1);
		//Save Yout
		y = ADC;
	} else if (joyStickName == 1) {
		//Switch to reading ADC2
		_delay_ms(10);
		ADMUX &= ~(1<<MUX0);
		ADMUX |= (1<<MUX1);
		//Save Xout
		x = ADC;
		//Switch to reading ADC3
		_delay_ms(10);
		ADMUX |= (1<<MUX0);
		ADMUX |= (1<<MUX1);
		//Save Yout
		y = ADC;
	}
	//Return position = (x, y)
	position.xy[0] = x;
	position.xy[1] = y;
	
	return position;
}

ISR(TIMER1_COMPA_vect) {
	//Increments score every 4 seconds
	score += 1;
	readyToUpdateScore = 1;
}

int main(void)
{
	//Initialize LCD, serial printing, ADC
	lcd_init();
	UART_init();
	Initialize();
	
		//Wait until game has begun
		/*while(gameHasBegun == 0) {
			sensorState1 = (PIND & (1<<PIND3));
			sprintf(str, "%d \n", sensorState1);
			UART_putstring(str);
			if ((sensorState1 == 0) && (lastState1 > 0)) {
				gameHasBegun = 1;	
			} 
			lastState1 = sensorState1;	
		}*/
	
	while(1) {
		//Load pinball machine welcome screen
		lcd_clear();
		lcd_write_word((uint8_t*) "Welcome to Game");
		lcd_goto_xy(1,0);
		lcd_write_word((uint8_t*) "Ready to Begin?");
	
		//Wait until game has begun
		while(gameHasBegun == 0) {
			sensorState1 = (PIND & (1<<PIND3));
			sprintf(str, "%d \n", sensorState1);
			UART_putstring(str);
			if ((sensorState1 == 0) && (lastState1 > 0)) {
				gameHasBegun = 1;	
			} 
			lastState1 = sensorState1;	
		}
	
		//When game has begun, display "Game has Begun" screen and score
		startGameTimer();
		lcd_clear();
		lcd_write_word((uint8_t*) "Game has Begun");
		lcd_goto_xy(1,0);
		sprintf(str, "Score: %d", score);
		lcd_write_word((uint8_t*) str);
	
		while(gameHasBegun) {
			//If Timer1 has overflowed, update score
			if (readyToUpdateScore) {
				lcd_goto_xy(1,0);
				sprintf(str, "Score: %d", score);
				lcd_write_word((uint8_t*) str);
				readyToUpdateScore = 0;
			}
			
			//Get position of Joystick0
			INT_ARRAY position0 = getJoyStickPosition(0);
			//Print (Xout, Yout) Joystick0 reading
			sprintf(str, "(%d, %d) \n", position0.xy[0], position0.xy[1]);
			UART_putstring(str);
			//Get position of Joystick1
			INT_ARRAY position1 = getJoyStickPosition(1);
			//Print (Xout, Yout) Joystick1 reading
			sprintf(str, "(%d, %d) \n", position1.xy[0], position1.xy[1]);
			UART_putstring(str);
			
			//Check if break beam sensor has been tripped
			sensorState2 = (PIND & (1<<PIND2));
			if ((sensorState2 == 0) && (lastState2 > 0)) {
				gameHasBegun = 0;
			}
			lastState2 = sensorState2;
		}
	
		//Game is over, display "Game Over" screen and final score
		lcd_clear();
		lcd_write_word((uint8_t*) "Game Over");
		//Disable interrupt for Timer 0
		TIMSK1 &= ~(1<<OCIE1A);
		//Delay for 10 seconds
		_delay_ms(10000);
	}
}


