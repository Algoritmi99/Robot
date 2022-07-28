/*
 * Driver.c
 *
 * Created: 6/21/2021 7:26:56 PM
 *  Author: Algoritmi
 */ 

#include "Driver.h"
#include "motor/motor.h"
#include <math.h>
#include "communication/communication.h"
#include "pathFollower/pathFollower.h"
#include <stdlib.h>
#include "Sensors/infrared.h"
#include <util/atomic.h>
#include "Sensors/encoder.h"
#include "Odometry.h"

int MVelocityR  = 0;
int MVelocityL = 0;

double speedV =  3000.0;

#define deltaT 0.150
#define AxleLengthL 155.


double absolute(double x){
	if (x < 0){
		return -x;
	}
	return x;
}

double scalarProduct(vector a, vector b){
	return (a.x * b.x) + (a.y * b.y);
}

double vectorLength(vector a){
	return sqrt((a.x * a.x) + (a.y * a.y));
}


//drive in a curve
void calculateDriveCommand(Pose_t* currentPose, const FPoint_t* lookahead){
	
	vector dir = {
		cos(currentPose->theta),
		sin(currentPose->theta)
	};
	
	vector dirToBe = {
		lookahead->x - currentPose->x,
		lookahead->y - currentPose->y
	};
	
	//double aCruzb = (dir.x * dirToBe.y) - (dir.y * dirToBe.x);
	//double deltaTheta = asin(fabs(aCruzb) / (vectorLength(dir) * vectorLength(dirToBe)));
	
	
	double aCruzb = (dir.x * dirToBe.y) - (dir.y * dirToBe.x);
	double arg = (dir.x*dirToBe.x + dir.y * dirToBe.y) / (vectorLength(dir) * vectorLength(dirToBe));
	double deltaTheta = acos(arg);
	
	
	if (aCruzb < 0){
		deltaTheta = -deltaTheta;
	}
	
	
	double factor = speedV / 1000;
	
	if (fabs(deltaTheta) <= (M_PI / 3)){
		speedV = 2000;
		factor *= 1.2;
	}
	else{
		speedV = 2000;
		factor = speedV / 1000;
	}
	
	double factorOfSpeedChange = 0.5 * AxleLengthL * tan(deltaTheta);
	//communication_log(LEVEL_INFO,"foc%d", (int16_t)factorOfSpeedChange);
	
	
	int16_t vr = (speedV + factor * factorOfSpeedChange);
	int16_t vl = (speedV - factor * factorOfSpeedChange);
	
	Motor_setVelocities(-vr, vl);
}


//No Curves. make angle right and go
void mazeRunner(Pose_t* pose, const FPoint_t* point){
	int16_t vr = speedV;
	int16_t vl = speedV;
	
	vector dir = {
		cos(pose->theta),
		sin(pose->theta)
	};
	
	vector dirToBe = {
		point->x - pose->x,
		point->y - pose->y
	};
	
	double aCruzb = (dir.x * dirToBe.y) - (dir.y * dirToBe.x);
	double arg = (dir.x*dirToBe.x + dir.y * dirToBe.y) / (vectorLength(dir) * vectorLength(dirToBe));
	double deltaTheta = acos(arg);
	
	if (aCruzb < 0){
		deltaTheta = -deltaTheta;
	}
	
	//communication_log(LEVEL_INFO, "theta %.3f, %.3f", deltaTheta * 180.0 / M_PI, arg);
	
	//Make angle right or go
	if (deltaTheta > 0.0523599){
		Motor_setVelocities(-(vr - 1000), -(vl - 1000));
		MVelocityL = -(vl - 1000);
		MVelocityR = -(vr - 1000);
	}
	else if (deltaTheta < -0.0523599){
		Motor_setVelocities((vr - 1000), (vl - 1000));
		MVelocityL = (vl - 1000);
		MVelocityR = (vr - 1000);
	}
	else{
		Motor_setVelocities(-vr, vl);
		MVelocityL = vl;
		MVelocityR = -vr;
	}
}

void goToCoordinate(Pose_t* pose, const FPoint_t* point){
	Point_t curr = {
		.x = (int16_t)pose->x+1,
		.y = (int16_t)pose->y+1
	};
	Point_t toGo = {
		.x = (int16_t)point->x,
		.y = (int16_t)point->y
	};
	
	Point_t* pointArray = malloc(2 * sizeof(Point_t));
	pointArray[0] = curr;
	pointArray[1] = toGo;
	
	const Path_t pathTogo = {
		.pathLength = 2,
		.points = pointArray
	};
	
	pathFollower_setNewPath(&pathTogo);
	free(pointArray);
	
	pathFollower_command(FOLLOWER_CMD_START);
	
}

bool lookRight(){
	return getDistanceRight() > (2 * 3.904);
}

bool lookLeft(){
	return getDistanceLeft() > (2 * 3.690);
}

bool lookFront(){
	return getDistanceFront() > (2 * 6.053);
}
