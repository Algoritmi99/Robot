/*
 * Odometry.c
 *
 * Created: 6/17/2021 5:52:26 PM
 *  Author: Algoritmi
 */ 

#include "Odometry.h"

Pose_t getNewPoseForwarding(Pose_t oldPose, int16_t dl, int16_t dr){
	int16_t d = (dl + dr) / 2;
	double deltaD = ((DistancePerTickR + DistancePerTickL) / 2) * d;
	
	double deltaX = deltaD * cos(oldPose.theta);
	double deltaY = deltaD * sin(oldPose.theta);
	
	Pose_t newPose = {oldPose.x + deltaX, oldPose.y + deltaY, oldPose.theta};
	
	return newPose;
}

Pose_t getNewPoseNotForwarding(Pose_t oldPose, int16_t dl, int16_t dr){
	double deltaDl = DistancePerTickL * dl;
	double deltaDr = DistancePerTickR * dr;
	
	double deltaTheta = (deltaDr - deltaDl) / AxleLength;
	double deltaX = ((deltaDr + deltaDl)/(deltaDr - deltaDl)) * (AxleLength / 2) * (sin(oldPose.theta + deltaTheta) - sin(oldPose.theta));
	double deltaY = ((deltaDr + deltaDl)/(deltaDr - deltaDl)) * (AxleLength / 2) * (cos(oldPose.theta) - cos(oldPose.theta + deltaTheta));
	
	double newTheta = oldPose.theta + deltaTheta;
	if (newTheta > M_PI){
		newTheta -= 2 * M_PI;
	}
	else if (newTheta < -M_PI){
		newTheta += 2 * M_PI;
	}
	
	Pose_t newPose = {oldPose.x + deltaX, oldPose.y + deltaY, newTheta};
		
	return newPose;
}

Pose_t getNewPose(Pose_t oldPose, int16_t dl, int16_t dr, bool forwarding){
	if (forwarding){
		return getNewPoseForwarding(oldPose, dl, dr);
	}
	return getNewPoseNotForwarding(oldPose, dl, dr);
}


