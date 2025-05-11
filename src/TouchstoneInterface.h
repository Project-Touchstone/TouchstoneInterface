// TouchstoneInterface.h : Include file for standard system include files,
// or project specific include files.

//External imports
#include <iostream>
#include <math.h>
#include <Eigen/Dense>
#include <thread>
#include <stdint.h>
#include <boost/asio.hpp>

//Local imports
#include "sensors/MagEncoder.h"
#include "sensors/IMU.h"
#include "sensors/MagTracker.h"
#include "sensors/Thimble.h"
#include "comms/SerialInterface.h"
#include "actuators/DRIFTMotor.h"
#include "actuators/DRIFTPlex.h"
#include "utils/Utils.h"
#include "utils/Timer.h"
#include "comms/HapticRenderServer.h"

#define NUM_MOTORS 4

#define SERIAL_PORT "\\\\.\\COM6"
#define BAUD_RATE 460800
#define TIMEOUT 1000

#define SERVER_PORT 8080

namespace SerialHeaders {
	//Headers from master to controller

	//Pings microcontroller
	#define PING 0x1
	//Servo power update
	#define SERVO_POWER 0x2

	//Headers from controller to master

	//Acknowledges ping      
	#define PING_ACK 0x1
	//PWM cycle start
	#define PWM_CYCLE 0x2  
	//Sends magnetic encoder data
	#define MAGENCODER_DATA 0xA0
	//Sends magnetic tracker data
	#define MAGTRACKER_DATA 0xA1
	//Sends IMU data
	#define IMU_DATA 0xA2              
}

uint8_t setup();
void generalScheduler();
void encoderCalibration();
void positionHoming();
void serialInterface();
void kinematicSolver();
void updateSim(bool printing);
void processing();