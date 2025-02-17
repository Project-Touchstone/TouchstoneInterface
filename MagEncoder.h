/*
  MagEncoder.h - Custom ring-based multi-magnet encoder implementation
  Created by Carson G. Ray
*/

#ifndef MagEncoder_h
#define MagEncoder_h

#include <math.h>
#include <mutex>
#include <Eigen/Geometry>

class MagEncoder {
    private:
        //Magnetic sensor data
        float sensorData[2] = {0, 0};
		//Maximum amplitudes
        float amplitudes[2] = {0, 0};
		//Phase offsets
        const float phases[2] = { -EIGEN_PI / 2, -EIGEN_PI };
		//Y Values
        float yVals[2];
		//Possible angles
        float angles[2][2];
		//Previous calculated angle
        float prevAngle = 0;
		//Integrated position
        float position = 0;
		//Offset from last reset
        float offset = 0;
		//Direction of encoder
        int8_t dir = 1;

		//Time of last velocity sample
		uint64_t sampleStart;
		//Position at last velocity sample
		float lastPosition = 0;

        //Mutex
		std::mutex mutex;
    public:
        void setDirection(int8_t dir);
        void updateData(float sensorData[2]);
        float relativePosition();
        float absolutePosition();
		float sampledVelocity();
        void reset();
};

#endif