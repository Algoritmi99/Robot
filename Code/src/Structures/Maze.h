/*
 * Maze.h
 *
 * Created: 7/1/2021 8:47:22 PM
 *  Author: Algoritmi
 */ 


#ifndef MAZE_H_
#define MAZE_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct{
	int x;
	int y;
} coordinate;

typedef struct{
	float x;
	float y;
	int marksD;
	int marksA;
	int marksW;
	int marksS;
	bool wallA;
	bool wallD;
	bool wallW;
	bool wallS;	
} cell;

extern cell maze[7][7];

coordinate coordLookup(int i, int j);
void mazeInit();




#endif /* MAZE_H_ */