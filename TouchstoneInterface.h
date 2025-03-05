// TouchstoneInterface.h : Include file for standard system include files,
// or project specific include files.

#include <iostream>

// TODO: Reference additional headers your program requires here.
#include "MagEncoder.h"
#include "SerialInterface.h"
#include "DRIFTMotor.h"
#include "DRIFTPlex.h"
#include <math.h>
#include <Eigen/Dense>
#include <thread>
#include <stdint.h>

#define NUM_MOTORS 3

#define SERIAL_PORT "\\\\.\\COM6"

namespace SerialHeaders {
    //Headers from master to controller

    //Pings microcontroller
    #define PING 0x1
    //Servo power update
    #define SERVO_POWER 0x2

    //Headers from controller to master
    
    //Acknowledges ping      
    #define PING_ACK 0x1
    //Sends sensor data
    #define SENSOR_DATA 0x2
    //PWM cycle start
    #define PWM_CYCLE 0xA
}

uint8_t setup();
void sleep(uint32_t ms);
void generalScheduler();
void encoderCalibration();
void positionHoming();
void serialInterface();
void kinematicSolver();
std::string toString(const Eigen::VectorXd mat);
void updateSim();