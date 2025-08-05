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
#include "comms/DataProtocol.h"

#define NUM_MOTORS 4

#define SERIAL_PORT "\\\\.\\COM6"
#define BAUD_RATE 460800
#define TIMEOUT 1000

#define SERVER_PORT 8080
#define SERVER_THREADS 1

namespace SerialHeaders {
	//Headers from master to controller

	//Pings microcontroller
	#define PING 0x1 // 0 bytes
	//Servo power update
	#define SERVO_POWER 0x2 // 3 bytes (1 byte motor id, 2 byte power value from 0 to 1)

	//Headers from controller to master

	//Acknowledges ping      
	#define PING_ACK 0x1 // 0 bytes
	//PWM cycle start
	#define PWM_CYCLE 0x2 // 0 bytes
	//Sends magnetic encoder data
	#define MAGENCODER_DATA 0xA0 // 5 bytes (1 byte sensor id, 4 byte Y and Z axes)
	//Sends magnetic tracker data
	#define MAGTRACKER_DATA 0xA1 // 7 bytes (1 byte sensor id, 6 byte X, Y and Z axes)
	//Sends IMU data
	#define IMU_DATA 0xA2 // 13 bytes (1 byte sensor id, 6 byte 3-axis accel data, 6 byte 3-axis gyro data)                                                                                           
}

namespace NetworkHeaders {
	// Headers from client to server
	#define NODE_DATA 0x1 // 0 bytes, 28 byte response (3 float cartesian position, 4 float quaternion orientation (i, j, k, w))
	#define FORCE_FEEDBACK 0x2 // 12 bytes (3 float force), 0 byte response
	#define COLLISION_FEEDBACK 0x3 // 28 bytes (3 float point, 3 float normal, 1 float time to collision seconds), 0 byte response

	//Headers from server to client
	#define ACK 0x1 // Followed by response data
	#define NACK 0x2 // 0 bytes
}

uint8_t setup();
void schedulerThread();
void calibration();
void homing();
void serialReadHandler(std::shared_ptr<DataProtocol> data);
void serialTimeoutHandler(std::shared_ptr<DataProtocol> data);
void serverRequestHandler(std::shared_ptr<DataProtocol> client);
void kinematicSolver();
void processingThread();

Quaterniond getTrueOrient();
Vector3d getAngularVelocity();