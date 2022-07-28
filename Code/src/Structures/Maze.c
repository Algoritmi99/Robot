/*
 * Maze.c
 *
 * Created: 7/1/2021 8:47:34 PM
 *  Author: Algoritmi
 */ 

#include "Maze.h"
#include <math.h>

cell maze[7][7];

coordinate coordLookup(int i, int j){
	coordinate out = {
		.x = round(253.3 * i),
		.y = round(253.3 * j)
	};
	return out;
}

cell makeCell(int i, int j){
	coordinate t = coordLookup(i - 3 , 3 - j);
	cell out = {t.x, t.y, 0, 0, 0, 0, false, false, false, false};
	return out;
}

void mazeInit(){
	for (int j = 0; j < 7; j++){
		for (int i = 0; i < 7; i++){
			maze[i][j] = makeCell(i, j);
		}
	}
}