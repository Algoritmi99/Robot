/*
 * bumper.c
 *
 * Created: 6/6/2021 11:37:19 AM
 *  Author: Algoritmi
 */ 

#include "bumper.h"

void bumperInit(){
	collisions = 0;
	DDRK &= ~((1 << PCINT22) | (1 << PCINT23));
	PCICR |= (1 <<PCIE2);
	PCMSK2 = ((1 << PCINT22) | (1 << PCINT23));
	
}

bool rightBumpOn(){
	if (PINK & (1 << rightBumpPin)){
		collisions++;
		return true;
	}
	return false;
}

bool leftBumpOn(){
	if (PINK & (1 << leftBumpPin)){
		collisions++;
		return true;
	}
	return false;
}

uint8_t rightBumpread(){
	if (rightBumpOn()){
		return 1;
	}
	return 0;
}

uint8_t leftBumpread(){
	if (leftBumpOn()){
		return 1;
	}
	return 0;
}

uint8_t getCollisions(){
	return collisions;
}

void incCollisions(){
	collisions++;
}
