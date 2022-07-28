/*
 * Odometry.h
 *
 * Created: 6/17/2021 5:52:42 PM
 *  Author: Algoritmi	
 */ 


#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#include "communication/packetTypes.h"
#include <stdbool.h>
#include <math.h>

#define DistancePerTickL 0.57035
#define DistancePerTickR 0.57055

#define AxleLength 155


typedef enum MovementStat {FORWARDING, TURNING, NOTMOVING} MovementStat;

Pose_t getNewPoseForwarding(Pose_t oldPose, int16_t dl, int16_t dr);
Pose_t getNewPoseNotForwarding(Pose_t oldPose, int16_t dl, int16_t dr);
Pose_t getNewPose(Pose_t oldPose, int16_t dl, int16_t dr, bool forwarding);


#endif /* ODOMETRY_H_ */