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
#include "comms/MinBiTCore.h"
#include "comms/MinBiTSerialClient.h"
#include "comms/MinBiTTcpServer.h"
#include "actuators/DRIFTMotor.h"
#include "actuators/DRIFTPlex.h"
#include "utils/Utils.h"
#include "utils/Timer.h"

#define NUM_MOTORS 4

#define SERIAL_PORT "\\\\.\\COM5"
#define BAUD_RATE 460800
#define TIMEOUT 1000

#define SERVER_PORT 8080
#define SERVER_THREADS 1

namespace SerialHeaders {
	//Headers from client (interface) to server (microcontroller)

	//Pings microcontroller
	#define PING 0x1 // 0 bytes, 0 byte response

	// Toggles configuration mode
	#define CONFIG 0x10 // 0 bytes, 0 byte response
	// Configures BusChain
	#define CONFIG_BUSCHAIN 0x11 // variable bytes, 0 byte response
	// Configures magnetic encoder
	#define CONFIG_MAG_ENCODER 0x12
	// Configures magnetic tracker
	#define CONFIG_MAG_TRACKER 0x13
	// Configures IMU
	#define CONFIG_IMU 0x14
	// Configures servo driver
	#define CONFIG_SERVO_DRIVER 0x15
	// Configures servo
	#define CONFIG_SERVO 0x16
	// Configures FOC motor
	#define CONFIG_FOC_MOTOR 0x17

	// Requests all sensor data
	#define SENSOR_DATA 0x2 // 0 bytes, variable byte response:
	//Sends magnetic encoder data
	// 4 bytes per sensor (2 bytes per Y and Z axes)
	//Sends magnetic tracker data
	// 6 bytes per sensor (2 bytes per X, Y and Z axes)
	//Sends IMU data
	// 13 bytes per sensor (6 byte 3-axis accel data, 6 byte 3-axis gyro data)                                                                              

	//Servo signal update
	#define SERVO_SIGNAL 0x30 // 3 bytes (1 byte servo id, 2 byte signal value from -1 to 1)
	//Sends FOC position target
	#define FOC_POSITION 0x31 // 5 bytes (1 byte motor id, 4 byte position value in radians)
	//Sends FOC velocity target
	#define FOC_VELOCITY 0x32 // 5 bytes (1 byte motor id, 4 byte velocity value in radians/s)
	//Sends FOC torque target
	#define FOC_TORQUE 0x33 // 5 bytes (1 byte motor id, 4 byte torque value in Nm)

	//Headers from server (microcontroller) to client (interface)

	//Acknowledge   
	#define ACK 0x1 // followed by response data
	//No acknowledge
	#define NACK 0x2 // 0 bytes
}

namespace NetworkHeaders {
	// Headers from client to server
	#define SEND_NODE_DATA 0x1 // 0 bytes, 28 byte response (3 float cartesian position, 4 float quaternion orientation (i, j, k, w))
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
void firmwareReadHandler(std::shared_ptr<MinBiTCore> protocol, std::shared_ptr<MinBiTCore::Request> request);
void appReadHandler(std::shared_ptr<MinBiTCore> protocol, std::shared_ptr<MinBiTCore::Request> request);
void kinematicSolver();
void processingThread();

Quaterniond getTrueOrient();
Vector3d getAngularVelocity();