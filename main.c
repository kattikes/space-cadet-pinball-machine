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
volatile int temp = 0;

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

void initbreakBeam(void) {
	//Set PD3 as input pin for break beam sensor 1
	DDRD &= ~(1<<DDD3);
	PORTD |= (1<<PORTD3);
	//Set PD2 as input pin for break beam sensor 2
	DDRD &= ~(1<<DDD2);
	PORTD |= (1<<PORTD2);
}

void initJoyStick(void) {	
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
	//Prescale Timer 0 by 1/1024
	TCCR0B |= (1<<CS00);
	TCCR0B &= ~(1<<CS01);
	TCCR0B |= (1<<CS02);
	//Set Timer 0 to normal
	TCCR0A &= ~(1<<WGM00);
	TCCR0A &= ~(1<<WGM01);
	TCCR0B &= ~(1<<WGM02);
	//Enable interrupt for Timer 0
	TIMSK0 |= (1<<OCIE0A);
	//Enable global interrupts
	sei();
}

/* Gets position of either Joystick0 or Joystick1. 
   Position is of the form (x, y), where 1 <= x, y <= 1023. 
   Top rightmost position is (1023, 1023). Bottom leftmost position is (1, 1).
   Centered/equilibrium position is about  (516, 517).
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
	position.xy[0] = x;
	position.xy[1] = y;
	//Return position = (x, y)
	return position;
}

//Increments score approximately every 4 seconds
ISR(TIMER0_COMPA_vect) {
	temp++;
	if (temp == 244) {
		score++;
		readyToUpdateScore = 1;
		temp = 0;
	}
}

int main(void)
{
	//Initialize LCD, serial printing, ADC
	lcd_init();
	UART_init();
	initJoyStick();
	
	while(1) {
		//Load pinball machine welcome screen
		lcd_clear();
		lcd_write_word((uint8_t*) "Welcome to Game");
		lcd_goto_xy(1,0);
		lcd_write_word((uint8_t*) "Ready to Begin?");
		
		//Initialize break beam sensor
		initbreakBeam();
		
		//Wait until game has begun
		while(gameHasBegun == 0) {
			//Check if top break beam sensor has been tripped
			sensorState1 = (PIND & (1<<PIND3));
			if ((sensorState1 == 0) && (lastState1 > 0)) {
				gameHasBegun = 1;	
			} 
			lastState1 = sensorState1;	
		}
	
		//When game has begun, display "Game has Begun" screen and score
		startGameTimer();
		score = 0;
		lcd_clear();
		lcd_write_word((uint8_t*) "Game has Begun");
		lcd_goto_xy(1,0);
		sprintf(str, "Score: %d", score);
		lcd_write_word((uint8_t*) str);
		
		//Increment score, read joystick, move servo
		while(gameHasBegun == 1) {
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
			/*sprintf(str, "(%d, %d) \n", position0.xy[0], position0.xy[1]);
			UART_putstring(str);*/
			//Get position of Joystick1
			INT_ARRAY position1 = getJoyStickPosition(1);
			//Print (Xout, Yout) Joystick1 reading
			/*sprintf(str, "(%d, %d) \n", position1.xy[0], position1.xy[1]);
			UART_putstring(str);*/
			
			
			/* Below is just example code, which just determines if the 
			   joysticks have been pushed to their topmost y position, 
			   then the respective servo motor is triggered. Feel free
			   to implement this section (including the conditional 
			   statements) as you wish
			*/
			/*
			if (position0.xy[1] > 700) {
				//Move right servo motor
			}
			if (position1.xy[1] > 700) {
				//Move left servo motor
			}
			*/
			
			//Re-initialize break beam sensor
			initbreakBeam();
			//Check if bottom break beam sensor has been tripped
			lastState2 = sensorState2;
			sensorState2 = (PIND & (1<<PIND2));
			//sprintf(str, "%d \n", sensorState2);
			//UART_putstring(str);
			if ((sensorState2 == 0) && (lastState2 > 0)) {
				gameHasBegun = 0;
			}
		}
	
		//Game is over, display "Game Over" screen and final score
		lcd_clear();
		lcd_write_word((uint8_t*) "Game Over");
		lcd_goto_xy(1,0);
		sprintf(str, "Score: %d", score);
		lcd_write_word((uint8_t*) str);
		
		//Delay for 10 seconds
		_delay_ms(10000);
	}
}

