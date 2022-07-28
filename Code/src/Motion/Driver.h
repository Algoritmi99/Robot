/*
 * Driver.h
 *
 * Created: 6/21/2021 7:26:44 PM
 *  Author: Algoritmi
 */ 


#ifndef DRIVER_H_
#define DRIVER_H_

extern int MVelocityR;
extern int MVelocityL;


#include "communication/packetTypes.h"
#include "pathFollower/pathFollower.h"
#include <stdbool.h>

typedef struct {
	double x;
	double y;
} vector;

void calculateDriveCommand(Pose_t* currentPose, const FPoint_t* lookahead);
double scalarProduct(vector a, vector b);
double vectorLength(vector a);
void mazeRunner(Pose_t* pose, const FPoint_t* point);
void goToCoordinate(Pose_t* pose, const FPoint_t* point);
bool lookRight();
bool lookleft();
bool lookfront();



#endif /* DRIVER_H_ */