/*
 * Kernel.c
 *
 * Created: 7/3/2021 10:58:11 AM
 *  Author: Algoritmi
 */ 

#include "Kernel.h"
#include "Motion/Driver.h"
#include <stdbool.h>
#include "communication/communication.h"
#include "pathFollower/pathFollower.h"
#include <stdlib.h>
#include "Sensors/infrared.h"
#include <math.h>
#include "motor/motor.h"

uint32_t indexOfChangeCheck;
coordinate lastCoordInMaze = {.x = 3, .y = 3};
coordinate currDestination = {.x = 3, .y = 3};
coordinate currDestinationCoords = {.x = 0, .y = 0};

GlobeWay getGlobeWay(Pose_t* pose){
	double theta = pose->theta;
	if (theta >= 1.44862 && theta <= 1.69297){
		return NORTH;
	}
	else if ((theta >= 3.01942 && theta <= M_PI) || (theta >= -M_PI && theta <= -3.01942)){
		return WEST;
	}
	else if (theta <= 0.122173 && theta >= -0.122173){
		return EAST;
	}
	else if (theta < -1.44862 && theta > -1.69297){
		return SOUTH;
	}
	//communication_log(LEVEL_INFO, "theta is: %f", theta);
	return UNCLEAR;
}

GlobeWay findGlobalWay(LocalWay dir, GlobeWay showing){
	GlobeWay out = UNCLEAR;
	if (showing == NORTH){
		if (dir == W){
			out = NORTH;
		}
		else if (dir == A){
			out = WEST;
		}
		else if (dir == D){
			out = EAST;
		}
		else if (dir == S){
			out = SOUTH;
		}
	}
	else if(showing == WEST){
		if (dir == W){
			out = WEST;
		}
		else if (dir == A){
			out = SOUTH;
		}
		else if (dir == D){
			out = NORTH;
		}
		else if (dir == S){
			out = EAST;
		}
	}
	else if (showing == SOUTH){
		if (dir == W){
			out = SOUTH;
		}
		else if (dir == A){
			out = EAST;
		}
		else if (dir == D){
			out = WEST;
		}
		else if (dir == S){
			out = NORTH;
		}
	}
	else if (showing == EAST){
		if (dir == W){
			out = EAST;
		}
		else if (dir == A){
			out = NORTH;
		}
		else if (dir == D){
			out = SOUTH;
		}
		else if (dir == S){
			out = WEST;
		}
	}
	return out;
}

void setWall(LocalWay dir, GlobeWay showing, coordinate* coordInMaze){
	GlobeWay toWall = findGlobalWay(dir, showing);
	communication_log(LEVEL_INFO, "toWall %d", toWall);
	if (toWall == NORTH){
		maze[coordInMaze->x][coordInMaze->y].wallW = true;
		if (coordInMaze->y - 1 >= 0){
			maze[coordInMaze->x][coordInMaze->y - 1].wallS = true;
		}
	}
	else if (toWall == WEST){
		maze[coordInMaze->x][coordInMaze->y].wallA = true;
		if (coordInMaze->x - 1 >= 0){
			maze[coordInMaze->x - 1][coordInMaze->y].wallD = true;
		}
	}
	else if (toWall == SOUTH){
		maze[coordInMaze->x][coordInMaze->y].wallS = true;
		if (coordInMaze->y + 1 <= 6){
			maze[coordInMaze->x][coordInMaze->y + 1].wallW = true;
		}
	}
	else if (toWall == EAST){
		maze[coordInMaze->x][coordInMaze->y].wallD = true;
		if (coordInMaze->x + 1 <= 6){
			maze[coordInMaze->x + 1][coordInMaze->y].wallA = true;
		}
	}
}

int marks(Pose_t* pose, coordinate* coordInMaze, LocalWay dirToSee){
	GlobeWay showing = getGlobeWay(pose);
	GlobeWay wayToSee = findGlobalWay(dirToSee, showing);
	int out = 0;
	if (wayToSee == NORTH){
		out = maze[coordInMaze->x][coordInMaze->y].marksW;
	}
	else if (wayToSee == WEST){
		out = maze[coordInMaze->x][coordInMaze->y].marksA;
	}
	else if (wayToSee == SOUTH){
		out = maze[coordInMaze->x][coordInMaze->y].marksS;
	}
	else if (wayToSee == EAST){
		out = maze[coordInMaze->x][coordInMaze->y].marksD;
	}
	return out;
}

void mark(Pose_t* pose, coordinate* coordInMaze, LocalWay dirToMark){
	GlobeWay showing = getGlobeWay(pose);
	GlobeWay wayToMark = findGlobalWay(dirToMark, showing);
	if (wayToMark == NORTH){
		maze[coordInMaze->x][coordInMaze->y].marksW++;
	}
	else if (wayToMark == WEST){
		maze[coordInMaze->x][coordInMaze->y].marksA++;
	}
	else if (wayToMark == SOUTH){
		maze[coordInMaze->x][coordInMaze->y].marksS++;
	}
	else if (wayToMark == EAST){
		maze[coordInMaze->x][coordInMaze->y].marksD++;
	}
}

int allMarksAround(coordinate* coordInMaze){
	return 
		maze[coordInMaze->x][coordInMaze->y].marksW +
		maze[coordInMaze->x][coordInMaze->y].marksA +
		maze[coordInMaze->x][coordInMaze->y].marksS +
		maze[coordInMaze->x][coordInMaze->y].marksD
	;
}

bool isWalled(Pose_t* pose, coordinate* coordInMaze, LocalWay dirToCheck){
	GlobeWay showing = getGlobeWay(pose);
	GlobeWay wayToCheck = findGlobalWay(dirToCheck, showing);
	bool out = true;
	if (wayToCheck == NORTH){
		out = maze[coordInMaze->x][coordInMaze->y].wallW;
	}
	else if (wayToCheck == WEST){
		out = maze[coordInMaze->x][coordInMaze->y].wallA;
	}
	else if (wayToCheck == SOUTH){
		out = maze[coordInMaze->x][coordInMaze->y].wallS;
	}
	else if (wayToCheck == EAST){
		out = maze[coordInMaze->x][coordInMaze->y].wallD;
	}
	return out;
}

LocalWay wayLessMarks(Pose_t* pose, coordinate* coordInMaze){
	LocalWay less = F;
	int lessMarks = 9999;
	
	int temp = marks(pose, coordInMaze, W);
	if (temp < lessMarks && !isWalled(pose, coordInMaze, W)){
		less = W;
		lessMarks = temp;
	}
	
	temp = marks(pose, coordInMaze, A);
	if (temp < lessMarks && !isWalled(pose, coordInMaze, A)){
		less = A;
		lessMarks = temp;
	}
	
	temp = marks(pose, coordInMaze, S);
	if (temp < lessMarks && !isWalled(pose, coordInMaze, S)){
		less = S;
		lessMarks = temp;
	}
	
	temp = marks(pose, coordInMaze, D);
	if (temp < lessMarks && !isWalled(pose, coordInMaze, D)){
		less = D;
		lessMarks = temp;
	}
	return less;
}

void setDestination(Pose_t* pose, coordinate* coordInMaze, LocalWay dirToGo){
	GlobeWay showing = getGlobeWay(pose);
	communication_log(LEVEL_INFO, "showing: %d", showing);
	communication_log(LEVEL_INFO, "dirToGo %d", dirToGo);
	GlobeWay wayToGo = findGlobalWay(dirToGo, showing);
	coordinate destination = {.x = coordInMaze->x, .y = coordInMaze->y};
	
	Point_t curr = {
		.x = maze[coordInMaze->x][coordInMaze->y].x,
		.y = maze[coordInMaze->x][coordInMaze->y].y
	};
	
	Point_t toGo;	
	
	 //int factorX = 0;
	 //int factorY = 0;
	 //
	
	if (wayToGo == NORTH){
		destination.y -= 1;
		//factorY = -50;
		curr.y += 10;
	}
	else if (wayToGo == WEST){
		destination.x -= 1;
		//factorX = 50;
		curr.x -= 10;
	}
	else if (wayToGo == SOUTH){
		destination.y += 1;
		//factorY = 50;
		curr.y -= 10;
	}
	else if (wayToGo == EAST){
		destination.x += 1;
		//factorX = -50;
		curr.x += 10;
	}
	
	if (destination.y < 0){
		toGo.x = maze[coordInMaze->x][coordInMaze->y].x;// + factorX;
		toGo.y = 1000;// + factorY;
	}
	else if (destination.y > 6){
		toGo.x = maze[coordInMaze->x][coordInMaze->y].x;// + factorX;
		toGo.y = -1000;// + factorY;
	}
	else if(destination.x < 0){
		toGo.y = maze[coordInMaze->x][coordInMaze->y].y;// + factorY;
		toGo.x = -1000;// + factorX;
	}
	else if (destination.x > 6){
		toGo.y = maze[coordInMaze->x][coordInMaze->y].y;// + factorY;
		toGo.x = 1000;// + factorX;
	}
	else{
		toGo.x = maze[destination.x][destination.y].x;// + factorX;
		toGo.y = maze[destination.x][destination.y].y;// + factorY;
	}
	
	//communication_log(LEVEL_INFO, "curr x: %d, y: %d", curr.x, curr.y);
	//communication_log(LEVEL_INFO, "dest x: %d, y: %d", toGo.x, toGo.y);
	
	Point_t* pointArray = malloc(2 * sizeof(Point_t));
	pointArray[0] = curr;
	pointArray[1] = toGo;
	
	const Path_t pathTogo = {
		.pathLength = 2,
		.points = pointArray
	};
	
	pathFollower_setNewPath(&pathTogo);
	free(pointArray);
	
	lastCoordInMaze.x = coordInMaze->x;
	lastCoordInMaze.y = coordInMaze->y;
	currDestination.x = destination.x;
	currDestination.y = destination.y;
	currDestinationCoords.x = toGo.x;
	currDestinationCoords.y = toGo.y;
	coordInMaze->x = destination.x;
	coordInMaze->y = destination.y;
}

bool haveGreed(Pose_t* pose, coordinate* coordInMaze, GlobeWay showing){
	if (coordInMaze->x < 6 &&
	coordInMaze->x > 0 &&
	coordInMaze->y < 6 &&
	coordInMaze->y > 0 ){
		return false;
	}
	if (coordInMaze->x == 6 && (!(maze[coordInMaze->x][coordInMaze->y].wallD))){
		communication_log(LEVEL_INFO, "wallD is false");
		if (showing == EAST){
			setDestination(pose, coordInMaze, W);
			return true;
		}
		if (showing == NORTH){
			setDestination(pose, coordInMaze, D);
			return true;
		}
		if (showing == WEST){
			setDestination(pose, coordInMaze, S);
			return true;
		}
		if (showing == SOUTH){
			setDestination(pose, coordInMaze, A);
			return true;
		}
	}
	if (coordInMaze->x == 0 && !maze[coordInMaze->x][coordInMaze->y].wallA){
		if (showing == EAST){
			setDestination(pose, coordInMaze, S);
			return true;
		}
		if (showing == NORTH){
			setDestination(pose, coordInMaze, A);
			return true;
		}
		if (showing == WEST){
			setDestination(pose, coordInMaze, W);
			return true;
		}
		if (showing == SOUTH){
			setDestination(pose, coordInMaze, D);
			return true;
		}
	}
	if (coordInMaze->y == 0 && !maze[coordInMaze->x][coordInMaze->y].wallW){
		if (showing == EAST){
			setDestination(pose, coordInMaze, A);
			return true;
		}
		if (showing == NORTH){
			setDestination(pose, coordInMaze, W);
			return true;
		}
		if (showing == WEST){
			setDestination(pose, coordInMaze, D);
			return true;
		}
		if (showing == SOUTH){
			setDestination(pose, coordInMaze, S);
			return true;
		}
	}
	if (coordInMaze->y == 6 && !maze[coordInMaze->x][coordInMaze->y].wallS){
		if (showing == EAST){
			setDestination(pose, coordInMaze, D);
			return true;
		}
		if (showing == NORTH){
			setDestination(pose, coordInMaze, S);
			return true;
		}
		if (showing == WEST){
			setDestination(pose, coordInMaze, A);
			return true;
		}
		if (showing == SOUTH){
			setDestination(pose, coordInMaze, W);
			return true;
		}
	}
	return false;
}

void startOpt(Pose_t* pose, bool* done, StateName* status){
	if (*done){
		*done = false;
		*status = CORE;
	}
}

void startOpt3(Pose_t* pose, bool* done, StateName* status){
	if (*done){
		*done = false;
		*status = CORE;
	}
}

void coreOpt(Pose_t* pose, bool* done, StateName* status, coordinate* coordInMaze){
	communication_log(LEVEL_INFO, "x: %d, y: %d", coordInMaze->x, coordInMaze->y);
	int8_t ways = 0;
	GlobeWay showing = getGlobeWay(pose);
	if (showing == UNCLEAR){
		*status = TURN;
		return;
	}
	if (coordInMaze->x < 0 || coordInMaze->x > 6 || coordInMaze->y < 0 || coordInMaze->x > 6){
		*status = END;
		communication_log(LEVEL_INFO, "I'm out!");
		return;
	}
	if (getDistanceFront() > (2 * 6.053)){
		ways++;
	}
	else{
		setWall(W, showing, coordInMaze);
		//communication_log(LEVEL_INFO, "setting wall front!");
	
	}
	if (getDistanceLeft() > (2 * 13.2)){
		ways++;
	}
	else{
		setWall(A, showing, coordInMaze);
	}
	if (getDistanceRight() > (9)){
		ways++;
	}
	else{
		setWall(D, showing, coordInMaze);
	}
	if (haveGreed(pose, coordInMaze, showing)){
		pathFollower_command(FOLLOWER_CMD_START);
		*status = MOVE;
		*done = false;
	}
	else if (ways > 1){
		*status = INPLACE;
		*done = false;
	} 
	else{
		*status = INWAY;
		*done = false;
	}
	communication_log(LEVEL_INFO, "WA %d, WD%d, WW%d, WS%d", maze[coordInMaze->x][coordInMaze->y].wallA,
	maze[coordInMaze->x][coordInMaze->y].wallD, maze[coordInMaze->x][coordInMaze->y].wallW, maze[coordInMaze->x][coordInMaze->y].wallS);
}

void inWayOpt(Pose_t* pose, bool* done, StateName* status, coordinate* coordInMaze){
	LocalWay toGo = W;
	//communication_log(LEVEL_INFO, "isWalledW: %d", isWalled(pose,coordInMaze, W));
	//communication_log(LEVEL_INFO, "isWalledN: %d", maze[coordInMaze->x][coordInMaze->y].wallW);
	
	if (isWalled(pose, coordInMaze, W)){
		if (!isWalled(pose, coordInMaze, A)){
			toGo = A;
		}
		else if (!isWalled(pose, coordInMaze, D)){
			toGo = D;
		}
		else{
			toGo = S;
		}
	}
	if (toGo != S && marks(pose, coordInMaze, toGo) >= 2){
		toGo = S;
	}
	
	setDestination(pose, coordInMaze, toGo);
	pathFollower_command(FOLLOWER_CMD_START);
	
	*status = MOVE;
	*done = false;
}

void inPlaceOpt(Pose_t* pose, bool* done, StateName* status, coordinate* coordInMaze){
	int marksAround = allMarksAround(coordInMaze);
	bool explorationOver = false;
	if (marksAround == 0){
		mark(pose, coordInMaze, S);
		LocalWay distance = S;
		if (!isWalled(pose, coordInMaze, W)){
			distance = W;
		}
		else if (!isWalled(pose, coordInMaze, A)){
			distance = A;
		}
		else if (!isWalled(pose, coordInMaze, D)){
			distance = D;
		}
		mark(pose,coordInMaze, distance);
		setDestination(pose, coordInMaze, distance);
		pathFollower_command(FOLLOWER_CMD_START);
	}
	else if (marksAround >= 1 && marks(pose, coordInMaze, S) == 0 && !isWalled(pose, coordInMaze, S)){
		mark(pose, coordInMaze, S);
		mark(pose, coordInMaze, S);
		setDestination(pose, coordInMaze, S);
		pathFollower_command(FOLLOWER_CMD_START);
	}

	else if (marks(pose, coordInMaze, wayLessMarks(pose, coordInMaze)) >= 2){
		explorationOver = true;
		communication_log(LEVEL_INFO, "Exploration is over. couldn't find a way out!");
	}
	else{
		LocalWay wayLeastMarks = wayLessMarks(pose, coordInMaze);
		mark(pose, coordInMaze, S);
		mark(pose, coordInMaze, wayLeastMarks);
		setDestination(pose, coordInMaze, wayLeastMarks);
		pathFollower_command(FOLLOWER_CMD_START);
	}
	
	if (explorationOver){
		*status = END;
	}
	else{
		*status = MOVE;
		*done = false;
	}
}

void moveOpt(Pose_t* pose, bool* done, StateName* status, coordinate* coordInMaze){
	bool reached = false;
	if (!pathFollower_getStatus()->enabled){
		*status = CORE;
		*done = false;
	}
	if ((pose->x > (currDestinationCoords.x - 120) && pose->y > (currDestinationCoords.y - 120)) &&
		(pose->x < (currDestinationCoords.x + 120) && pose->y < (currDestinationCoords.y + 120))){
		reached = true;
	}
	if (MVelocityR < 0 && MVelocityL > 0 && (getDistanceFront() < 3.5)){
		communication_log(LEVEL_INFO, "MoveOpt: L %d, R %d", MVelocityL, MVelocityR);
		pathFollower_command(FOLLOWER_CMD_PAUSE);
		Motor_stopAll();
		*status = CORE;
		*done = false;
		MVelocityL = 0;
		MVelocityR = 0;
		if ((coordInMaze->x != lastCoordInMaze.x || coordInMaze->y != lastCoordInMaze.y) && !reached){
			coordInMaze->x = lastCoordInMaze.x;
			coordInMaze->y = lastCoordInMaze.y;
		}
	}
}

void turnOpt(Pose_t* pose, bool* done, StateName* status){
	GlobeWay showing = getGlobeWay(pose);
	if (showing == UNCLEAR){
		Motor_setVelocities(1000, 1000);
	}
	else{
		*status = CORE;
		*done = false;
	}
}

void turnOpt2(Pose_t* pose, bool* done, StateName* status){
	GlobeWay showing = getGlobeWay(pose);
	if (showing == UNCLEAR){
		Motor_setVelocities(-1000, -1000);
	}
	else{
		*status = CORE;
		*done = false;
	}
}

void turnOpt3 (Pose_t* pose, bool* done, StateName* status){
	GlobeWay showing = getGlobeWay(pose);
	if (showing == UNCLEAR){
		Motor_setVelocities( 1000, 1000);
	}
	else{
		*status = CORE;
		*done = false;
	}
}