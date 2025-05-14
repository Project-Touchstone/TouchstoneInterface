/*
  MagEncoder.h - Custom ring-based multi-magnet encoder implementation
  Created by Carson G. Ray
*/

#ifndef MagEncoder_h
#define MagEncoder_h

//External imports
#include <math.h>
#include <mutex>
#include <Eigen/Geometry>
#include <stdint.h>
#include <chrono>

//Local imports
#include "../utils/Timer.h"

using namespace std::chrono;

class MagEncoder {
    private:
        //Raw sensor data
        std::array<int16_t, 2> rawData;
        //Sensor data mutex
        std::mutex dataMutex;
        //Sensor data multiplier
        const double magSensorMultiplier = 0.098;
		//Maximum amplitudes
        double amplitudes[2] = {0, 0};
		//Phase offsets
        const double phases[2] = { -(double)EIGEN_PI / 2, -(double)EIGEN_PI };
		//Y Values
        double yVals[2];
		//Possible angles
        double angles[2][2];
		//Previous calculated angle
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
        void storeRawData(const std::array<int16_t, 2>& data);
        void updateData();
        double relativePosition();
        double absolutePosition();
		double sampledVelocity();
        void reset();
};

#endif