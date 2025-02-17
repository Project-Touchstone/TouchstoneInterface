// TouchstoneInterface.h : Include file for standard system include files,
// or project specific include files.

#pragma once

#include <iostream>

// TODO: Reference additional headers your program requires here.
#include "MagEncoder.h"
#include "SerialInterface.h"
#include "DRIFTMotor.h"
#include "DRIFTPlex.h"
#include <math.h>
#include <Eigen/Dense>

namespace SerialHeaders {
    //Commands from master to controller

    //Pings microcontroller
    #define PING 0x1
    //Requests sensor data from microcontroller
    #define REQUEST_DATA 0x2
    //Servo power update
    #define SERVO_POWER 0x3

    //Commands from controller to master
    //Acknowledges ping      
    #define PING_ACK 0x1
    //Sends sensor data
    #define SENSOR_DATA 0x2
    //Sensor data ready
    #define DATA_READY 0xA                                                                                                        
}