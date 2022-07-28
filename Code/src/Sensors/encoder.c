/*
 * encoder.c
 *
 * Created: 6/6/2021 11:33:15 AM
 *  Author: Algoritmi
 */ 

#include "encoder.h"
#include "stdlib.h"

int16_t leftEncPulses;
int16_t rightEncPulses;

void encoderInit(){
	
	DDRB = 0x00;
	
	PORTB = 0xff;
	
	PCICR |= (1<<PCIE0);
	PCMSK0 |= (1<<PCINT7) | (1<<PCINT6) | (1<<PCINT5) | (1<<PCINT4);
	
	statInit();
	
	leftEncPulses = 0;
	rightEncPulses = 0;
}

void statInit(){
	lastStatPinb = PINB;
}

enum direction rightEncFigure(){
	uint8_t pinb = PINB;
	uint8_t index = (
	(((lastStatPinb & (1 << rightEncoderA)) != 0) << 3) + 
	(((lastStatPinb & (1 << rightEncoderB)) != 0) << 2) +
	(((pinb & (1 << rightEncoderA)) != 0) << 1) +
	((pinb & (1 << rightEncoderB)) != 0)
	);
	
	rightEncPulsed = index == 4 || index == 8;
	
	return checkUpTable[index];
}

enum direction leftEncFigure(){
	uint8_t currStatPinb = PINB;
	uint8_t index = (
	(((lastStatPinb & (1 << leftEncoderA)) != 0) << 3) +
	(((lastStatPinb & (1 << leftEncoderB)) != 0) << 2) +
	(((currStatPinb & (1 << leftEncoderA)) != 0) << 1) +
	((currStatPinb & (1 << leftEncoderB)) != 0)
	);
	
	leftEncPulsed = index == 4 || index == 8;
	
	return checkUpTable[index];
}

