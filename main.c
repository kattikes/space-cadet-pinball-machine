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

// variables and arrays for the motors
int size = 112;
volatile int pos_bool = 0; // boolean
volatile int left_pressed = 0; // boolean
volatile int right_pressed = 0; // boolean
volatile int lookup_count = 0;
int left_pos[112] =
{
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32
};
int right_pos[112] =
{
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32
};
int one_pos[112] = 
{
	64, 64, 64, 64,
	64, 64, 64, 64,
	64, 64, 64, 64,
	64, 64, 64, 64,
	64, 64, 64, 64,
	64, 64, 64, 64,
	64, 64, 64, 64,
	64, 64, 64, 64,
	64, 64, 64, 64,
	64, 64, 64, 64,
	64, 64, 64, 64,
	64, 64, 64, 64,
	64, 64, 64, 64,
	64, 64, 64, 64,
	64, 64, 64, 64,
	64, 64, 64, 64,
	64, 64, 64, 64,
	64, 64, 64, 64,
	64, 64, 64, 64,
	64, 64, 64, 64,
	64, 64, 64, 64,
	64, 64, 64, 64,
	64, 64, 64, 64,
	64, 64, 64, 64,
	64, 64, 64, 64,
	64, 64, 64, 64,
	64, 64, 64, 64,
	64, 64, 64, 64
};
int zero_pos[112] =
{
	32, 32, 32, 32, 
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32,
	32, 32, 32, 32
};

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
	
	///////   Timer 1 Setup ///////
	
	DDRB |= (1<<DDB1); // GPIO Pins Setup. OC1A
	DDRB |= (1<<DDB5); // SCK
	
	DDRB |= (1<<DDB2); // OC1B for motor 2
		
	TCCR1B |= (1<<CS12); // Prescale of 256
	TCCR1B &= ~(1<<CS10);
		
	// Phase correct PWM
	// 8 bit, Count to TOP
	TCCR1A |= (1<<WGM10);
		
	OCR1A = 5; // 50;
	OCR1B = 5; // for motor 2
		
	// Enable Timer Overflow Interrupt
	TIMSK1 |= (1<<TOIE1);
	TIFR1 |= (1<<TOV1);
		
	// Non-inverting mode
	// clear on compare match
	TCCR1A |= (1<<COM1A1);
	TCCR1A |= (1<<COM1B1); // for motor 2
	
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

// Interrupt for the motors
ISR(TIMER1_OVF_vect)
{
	PORTB ^= (1<<PORTB5);
	if (lookup_count < size)
	{
		OCR1A = left_pos[lookup_count];
		OCR1B = right_pos[lookup_count]; // for motor 2
		lookup_count++;
	}
	else
	{
		lookup_count = 0;
		if (pos_bool == 1)
		{
			pos_bool = 0;
			for (int i = 0; i < size; i++)
			{
				left_pos[i] = zero_pos[i];
				right_pos[i] = zero_pos[i];
			}
		}
		else
		{
			if(left_pressed || right_pressed)
			{
				pos_bool = 1;
				if(left_pressed)
				{
					for (int i = 0; i < size; i++)
					{
						left_pos[i] = one_pos[i];
						right_pos[i] = zero_pos[i];
					}
				}
				if(right_pressed)
				{
					for (int i = 0; i < size; i++)
					{
						left_pos[i] = zero_pos[i];
						right_pos[i] = one_pos[i];
					}
				}
				left_pressed = 0;
				right_pressed = 0;
			}
			else
			{
				for (int i = 0; i < size; i++)
				{
					left_pos[i] = zero_pos[i];
					right_pos[i] = zero_pos[i];
				}
			}
		}
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
			if (position0.xy[1] > 700) {
				//Move right servo motor
				right_pressed = 1;
			}
			if (position1.xy[1] > 700) {
				left_pressed = 1;
			}
			
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

