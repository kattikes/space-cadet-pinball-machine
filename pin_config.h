/*
 * pin_config.h
 *
 * Author : Hossam Elbahrawy
 */ 

#define F_CPU 16000000UL

#ifndef PIN_CONFIG_H_
#define PIN_CONFIG_H_

#include <avr/io.h>
#include <util/delay.h>

#define DATA_BUS	PORTD
#define CTL_BUS		PORTB
#define DATA_DDR	DDRD
#define CTL_DDR		DDRB
#define LCD_D4			4
#define LCD_D5			5
#define LCD_D6			6
#define LCD_D7			7
#define LCD_EN			3
#define	LCD_RS			4




#endif /* PIN_CONFIG_H_ */