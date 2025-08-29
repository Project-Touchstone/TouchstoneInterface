/*
  MagEncoder.h - Rotary encoder driver
  Created by Carson G. Ray
*/

#ifndef MagEncoder_h
#define MagEncoder_h

//External imports
#include <math.h>
#include <mutex>
#include <Eigen/Dense>
#include <stdint.h>
#include <chrono>

//Local imports
#include "../utils/Timer.h"

using namespace std::chrono;

class MagEncoder {
    private:
        //Sensor data mutex
        std::mutex dataMutex;
        //Sensor data multiplier
        const double rawToRadians = 2*EIGEN_PI/4096;
		//Previous angle
        double prevAngle = 0;
		//Integrated position
        double position = 0;
		//Offset from last reset
        double offset = 0;
		//Direction of encoder
        int8_t dir = 1;

		//Timer for velocity sampling
        Timer timer;
		//Position at last velocity sample
		double lastPosition = 0;

    public:
        MagEncoder();
        void setDirection(int8_t dir);
        void updateData(uint16_t rawAngle);
        virtual double relativePosition();
        virtual double absolutePosition();
		virtual double sampledVelocity();
        virtual void reset();
};

#endif