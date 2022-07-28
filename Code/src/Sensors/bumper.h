/*
 * bumper.h
 *
 * Created: 6/6/2021 11:37:37 AM
 *  Author: Algorimi
 */ 



#ifndef BUMPER_H_
#define BUMPER_H_

#include "avr/io.h"
#include "stdbool.h"
#include <avr/iom1280.h>
#include <util/delay.h>

//Port D 1 und 0
#define rightBumpPin PCINT23
#define leftBumpPin PCINT22

uint8_t collisions;

void bumperInit();
bool rightBumpOn();
bool leftBumpOn();
uint8_t rightBumpread();
uint8_t leftBumpread();
uint8_t getCollisions();
void incCollisions();





#endif /* BUMPER_H_ */