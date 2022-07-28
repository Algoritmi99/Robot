/*
 * infrared.c
 *
 * Created: 6/6/2021 11:32:03 AM
 *  Author: Algoritmi
 */ 

#include <cfg/Infrared/Infrared_cfg.h>
#include <io/adc/adc.h>
#include <sensors/infrared.h>

double getDistanceFront(){
	return (distanceParamAFront + (distanceParamBFront / ADC_getFilteredValue(FrontChannel)));
	//return (ADC_getFilteredValue(FrontChannel));
	
}

double getDistanceLeft(){
	return (distanceParamALeft + (distanceParamBLeft / ADC_getFilteredValue(LeftChannel)));
	//return (distanceParamALeft / (ADC_getFilteredValue(LeftChannel) + distanceParamBLeft) + distanceParamCLeft) - LeftOffset;
	//return (ADC_getFilteredValue(LeftChannel));
	
}

double getDistanceRight(){
	return (distanceParamARight + (distanceParamBRight / ADC_getFilteredValue(RightChannel)));
	//return (distanceParamARight / (ADC_getFilteredValue(RightChannel) + distanceParamBRight) + distanceParamCRight) - RightOffset;
	//return (ADC_getFilteredValue(RightChannel));
	
}