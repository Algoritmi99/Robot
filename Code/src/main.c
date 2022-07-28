#include <io/uart/uart.h>
#include <communication/communication.h>
#include <tools/timeTask/timeTask.h>
#include <tools/powerSaver.h>
#include <io/led/led.h>
#include <motor/motor.h>
#include <util/atomic.h>
#include <stdio.h>
#include <avr/io.h>                 // AVR IO ports
#include <avr/interrupt.h>          // AVR Interrupts
#include <avr/pgmspace.h>           // AVR Program Space Utilities
#include <math.h>                   // Math functions and constants
#include <inttypes.h>               // Integer type conversions for printf, scanf and alike
#include <stdbool.h>
#include "Sensors/bumper.h"
#include "Sensors/infrared.h"
#include "Sensors/encoder.h"
#include "Motion/Odometry.h"
#include "Motion/Driver.h"
#include "io/adc/adc.h"
#include "Structures/Maze.h"
#include "Structures/Kernel.h"


/*
 *******************************************************************************
 * PRIVATE VARIABLES
 *******************************************************************************
 */

// robot's pose, initial pose: x=200, y=0, theta=PI/2 (Northern direction)
Pose_t pose = { 0.0f, 0.0f, M_PI_2 };
MovementStat movementstat;

bool inMaze = true;

coordinate coord = {.x = 3, .y = 3};

StateName stat = START;
StateName statDummy = END;

bool doneHere = false;


FPoint_t p = {.x = -250, .y = -250};


/*
 *******************************************************************************
 * ISRs for Interrupt Handling
 *******************************************************************************
 */

// Bumpers
ISR(PCINT2_vect){
	bool R = rightBumpOn();
	bool L = leftBumpOn();
	if(R){
		//incCollisions();
		communication_log(LEVEL_INFO, "right bumper was pressed!");
		//write code to solve the collision problem
	}
	else if (L){
		//incCollisions();
		communication_log(LEVEL_INFO, "left bumper was pressed!");
		//write code to solve the collision problem
	}
	if (R || L){
		Motor_stopAll();
	}

}

//Encoders
int16_t dummyPulsesL = 0;
int16_t dummyPulsesR = 0;
ISR(PCINT0_vect){
	enum direction leftDir = leftEncFigure();
	enum direction rightDir = rightEncFigure();
	
	if (leftDir == FORWARD){
		//communication_log(LEVEL_INFO, "LF");
		if (leftEncPulsed){
			leftEncPulses++;
			leftEncPulsed = false;
			dummyPulsesL++;
		}
	}
	
	else if (leftDir == BACKWARD){
		//communication_log(LEVEL_INFO, "LB");
		//communication_log(LEVEL_INFO, "%d",leftEncPulsed);
		if (leftEncPulsed){
			leftEncPulses--;
			leftEncPulsed = false;
			dummyPulsesL--;
			
		}
	}
	
	if (rightDir == FORWARD){
		//communication_log(LEVEL_INFO, "RF");
		//communication_log(LEVEL_INFO, "%d",rightEncPulsed);
		if (rightEncPulsed){
			rightEncPulses++;
			rightEncPulsed = false;
			dummyPulsesR++;
			
		}
	}
	
	else if (rightDir == BACKWARD){
		//communication_log(LEVEL_INFO, "RB");
		//communication_log(LEVEL_INFO, "%d",rightEncPulsed);
		if (rightEncPulsed){
			rightEncPulses--;
			rightEncPulsed = false;
			dummyPulsesR--;
			
		}
	}
	
	//if (leftDir == UNDEF || rightDir == UNDEF){
		////communication_log(LEVEL_INFO, "UD");
	//}
	
	
	statInit();
}


/*
 *******************************************************************************
 * PRIVATE FUNCTIONS
 *******************************************************************************
 */



// callback function for communication channel CH_IN_DEBUG (Debug View in HWPCS)
static void commDebug(const uint8_t* packet, const uint16_t size) {
    communication_log(LEVEL_FINE, "received %" PRIu16 " bytes", size);
}


// callback function for communication channel CH_IN_USER_COMMAND (User Command View in HWPCS)
static void commUserCommand(const uint8_t* packet, const uint16_t size) {
    UserCommand_t* cmd = (UserCommand_t*) packet;
    switch (cmd->id) {
		case 0: // command ID 0: stop motors
			Motor_stopAll();
			movementstat = NOTMOVING;
			break;
		case 1: // command ID 1: turn on spot
			Motor_setVelocities(-1500, 0);
			movementstat = TURNING;
			break;
		case 2: // command ID 2: drive forwards
			Motor_setVelocities(-3000, 3000);
			movementstat = FORWARDING;
			break;
		case 3: // command ID 3 : Switch navigation mode
			inMaze = !inMaze;
		
			if (inMaze){
				communication_log(LEVEL_INFO, "Switched to Labyrinth Mode!");
				pathFollower_setLookaheadDistance(50.0);
			}
			else{
				communication_log(LEVEL_INFO, "Switched to Path Follow Mode!");
				pathFollower_setLookaheadDistance(20.0);
				
			}
			
			break;
		case 4: //Go to sat point
			goToCoordinate(&pose, &p);
			break;
		case 5: //Explore Labyrinth
			doneHere = true;
			communication_log(LEVEL_INFO, "Labyrinth Exploration started!");
			break;
		case 6: //reset pose
			pose.theta = M_PI_2;
			pose.x = 10;
			pose.y = 10;
		case 7:
			communication_log(LEVEL_INFO, "x:%d , y:%d", (int16_t)maze[4][4].x, (int16_t)maze[4][4].y);
		case 8: //Stop lab Exploration
			pathFollower_command(FOLLOWER_CMD_RESET);
			coord.x = 3;
			coord.y = 3;
			stat = START;
			Motor_stopAll();
			doneHere = false;
    }
}

void aprilFools(const uint8_t* packet, const uint16_t size){
	Pose_t* payload = (Pose_t*) packet;
	pose.x = payload->x;
	pose.y = payload->y;
	pose.theta = payload->theta;
}


// initialization
static void init(void) {
    powerSaver_init(); // must be the first call!
    LED_init();
    uart_init();
    communication_init();
	
	movementstat = NOTMOVING;
	
	communication_init();
	

    // register communication callback functions which are executed by
    // communication_readPackets() in main loop when a packet is received from
    // HWPCS on the corresponding communication channel
	communication_setCallback(CH_IN_DEBUG, commDebug);
	communication_setCallback(CH_IN_USER_COMMAND, commUserCommand);

    Motor_init();
    timeTask_init();
	ADC_init(false);
	bumperInit();
	encoderInit();
	pathFollower_init();
	pathFollower_setLookaheadDistance(50.0);
	
	mazeInit();
	
	communication_setCallback(CH_IN_POSE, aprilFools);
    // global interrupt enable
    sei();
}


int main(void) {
    init();

    communication_log_P(LEVEL_INFO, PSTR("Booted"));
	

    // do forever
    for (;;) {

        // TODO: do some other stuff

        TIMETASK(LED_TASK, 500) { // execute block approximately every 500ms
            LED2_TOGGLE();
        }

        TIMETASK(TELEMETRY_TASK, 300) { // execute block approximately every 300ms
            // send telemetry data to HWPCS
            Telemetry_t telemetry;
            telemetry.bumpers.value = 0; // initialize with zero
			telemetry.bumpers.value = 0;
            telemetry.bumpers.bitset.bit1 = 1;
			telemetry.bumpers.bitset.bit0 = 1;
            telemetry.contacts = getCollisions();
            telemetry.encoder1 = leftEncPulses;
            telemetry.encoder2 = rightEncPulses;
			//communication_log(LEVEL_INFO, "front %d", (int16_t)(getDistanceFront() * 1000));
			//communication_log(LEVEL_INFO, "right%f", (getDistanceRight()));
			//communication_log(LEVEL_INFO, "left%f", (getDistanceLeft()));
            telemetry.infrared1 = (int16_t)getDistanceFront();
            telemetry.infrared2 = (int16_t)getDistanceLeft();
            telemetry.infrared3 = (int16_t)getDistanceRight();
            telemetry.infrared4 = 0;
            telemetry.infrared5 = 0;
            telemetry.user1 = 20;
            telemetry.user2 = 42.42f;
            communication_writePacket(CH_OUT_TELEMETRY, (uint8_t*)&telemetry, sizeof(telemetry));
        }

        TIMETASK(POSE_TASK, 20) { // execute block approximately every 25ms
			int16_t pulsesRight;
			int16_t pulsesLeft;
			
			ATOMIC_BLOCK(ATOMIC_FORCEON){
				pulsesRight = rightEncPulses;
				pulsesLeft = leftEncPulses;
				rightEncPulses = 0;
				leftEncPulses = 0;
			}
			
			
			if (pulsesLeft == (-pulsesRight)){
				pose = getNewPoseForwarding(pose, pulsesLeft, (-pulsesRight));
			}
			else {
				pose = getNewPoseNotForwarding(pose, pulsesLeft, (-pulsesRight));
			}
			communication_writePacket(CH_OUT_POSE, (uint8_t*)&pose, sizeof(pose));
			//communication_log(LEVEL_INFO, "R %d", dummyPulsesR);
			//communication_log(LEVEL_INFO, "L %d", dummyPulsesL);
			
		}
			
		TIMETASK(FOLLOWER_TASK, 25) {
			const PathFollowerStatus_t* pathFollower_status = pathFollower_getStatus();
			if (pathFollower_status->enabled) {
				if (pathFollower_update(&pose)){
					if (!inMaze){
						calculateDriveCommand(&pose, &(pathFollower_status->lookahead));
					}
					else{
						mazeRunner(&pose, &(pathFollower_status->lookahead));
					}
				}
				else{
					MVelocityL = 0;
					MVelocityR = 0;
					Motor_stopAll();
				}
				communication_writePacket(0x03,(uint8_t*)pathFollower_status,sizeof(PathFollowerStatus_t));
				//sendPathFollowerStatus(); // send pathFollower_status on channel CH_OUT_PATH_FOLLOW_STATUS
			}
		}
		
		TIMETASK(APRIL_TASK, 25000){
			AprilTagType_t aprilRequest = APRIL_TAG_MAIN;
			communication_writePacket(CH_IN_POSE, &aprilRequest, sizeof(aprilRequest));
			
		}
		
				

        // poll receive buffer (read and parse all available packets from UART buffer)
        // and execute registered callback functions
        communication_readPackets();
		
		//The code for OS comes here
		switch(stat){
			case START:
				startOpt(&pose, &doneHere, &stat);
				break;
			case CORE:
				coreOpt(&pose, &doneHere, &stat, &coord);
				break;
			case INWAY:
				inWayOpt(&pose, &doneHere, &stat, &coord);
				break;
			case INPLACE:
				inPlaceOpt(&pose, &doneHere, &stat, &coord);
				break;
			case MOVE:
				moveOpt(&pose, &doneHere, &stat, &coord);
				break;
			case TURN:
				turnOpt(&pose, &doneHere, &stat);
				break;
			case END:
				stat = START;
				doneHere = false;
		}
		if (stat != statDummy){
		communication_log(LEVEL_INFO, "Status changed to: %d!", stat);
		statDummy = stat;
		}
		
	}
		return 0;
	
}
