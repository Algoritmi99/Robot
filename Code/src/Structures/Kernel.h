/*
 * Kernel.h
 *
 * Created: 7/3/2021 10:57:59 AM
 *  Author: Algoritmi
 */ 


#ifndef KERNEL_H_
#define KERNEL_H_

#include "communication/packetTypes.h"
#include "Maze.h"

extern uint32_t indexOfChangeCheck;

typedef enum{
	NORTH,
	SOUTH,
	EAST,
	WEST,
	UNCLEAR	
} GlobeWay;

typedef enum{
	W,	//forward
	A,	//left
	S,	//backward
	D,	//right
	F	//Fault
} LocalWay;

typedef enum {
	START,
	CORE,
	TURN,
	MOVE,
	INWAY,
	INPLACE,
	END
} StateName;

//typedef void (*Operation)(Pose_t* pose, State* currStat);

typedef struct  {
	StateName state;
	void (*operation)(Pose_t* pose);
	bool doneHere;
} State;

void startOpt(Pose_t* pose, bool* done, StateName* status);
void coreOpt(Pose_t* pose, bool* done, StateName* status, coordinate* coordInMaze);
void turnOpt(Pose_t* pose, bool* done, StateName* status);
void moveOpt(Pose_t* pose, bool* done, StateName* status, coordinate* coordInMaze);
void inWayOpt(Pose_t* pose, bool* done, StateName* status, coordinate* coordInMaze);
void inPlaceOpt(Pose_t* pose, bool* done, StateName* status, coordinate* coordInMaze);
void endOpt(Pose_t* pose, bool* done, StateName* status);

//State start = {START, &startOpt, false};
//State core = {CORE, &coreOpt, false};
//State turn = {TURN, &turnOpt, false};
//State move = {MOVE, &moveOpt, false};
//State inWay = {INWAY, &inWayOpt, false};
//State inPlace = {INPLACE, &inPlaceOpt, false};
//State end = {END, &endOpt};
	
void updateState(State* currStat);
State getState(StateName state);


#endif /* KERNEL_H_ */