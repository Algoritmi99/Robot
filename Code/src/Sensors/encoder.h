/*
 * encoder.h
 *
 * Created: 6/6/2021 11:33:32 AM
 *  Author: Algoritmi
 */ 

#include "avr/io.h"
#include "stdlib.h"
#include "stdbool.h"

//Port B 7,6,5,4
#ifndef ENCODER_H_
#define ENCODER_H_



#define rightEncoderA PCINT7 //PB7
#define rightEncoderB PCINT6 //PB6
#define leftEncoderA PCINT5  //PB5
#define leftEncoderB PCINT4  //PB4


typedef enum direction {UNDEF, BACKWARD, FORWARD} direction;


volatile uint8_t lastStatPinb;

volatile bool rightEncPulsed;
volatile bool leftEncPulsed;

extern int16_t leftEncPulses;
extern int16_t rightEncPulses;


static const direction checkUpTable[] = {
	UNDEF,					//0000,
	BACKWARD,				//0001,
	FORWARD,				//0010,
	UNDEF,					//0011,
	FORWARD,				//0100,
	UNDEF,					//0101,
	UNDEF,					//0110,
	BACKWARD,				//0111,
	BACKWARD,				//1000,
	UNDEF,					//1001,
	UNDEF,					//1010,
	FORWARD,				//1011,
	UNDEF,					//1100,
	FORWARD,				//1101,
	BACKWARD,				//1110,
	UNDEF,					//1111
};


//RA RB LA LB


void encoderInit(); // set pins and statinit
void statInit(); // set last stat eq currstat
enum direction rightEncFigure(); //get dir
enum direction leftEncFigure(); //get dir




#endif /* ENCODER_H_ */