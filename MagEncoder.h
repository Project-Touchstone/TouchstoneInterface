/*
  MagEncoder.h - Custom ring-based multi-magnet encoder implementation
  Created by Carson G. Ray
*/

#ifndef MagEncoder_h
#define MagEncoder_h

#include "Arduino.h"
#include <Tlv493d.h>
#include <math.h>
#include "freertos/task.h"

class MagEncoder {
    private:
		//Magnetic sensor object
        Tlv493d magSensor;

        //Magnetic sensor data
        float sensorData[2] = {0, 0};
		//Maximum amplitudes
        float amplitudes[2] = {0, 0};
		//Phase offsets
        const float phases[2] = {-PI/2, -PI};
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

        //Spinlock for RTOS
        portMUX_TYPE* spinlock;

        //Mutex for encoder data updates
        SemaphoreHandle_t mutex;

    public:
        MagEncoder();
        bool begin();
        void setUnitsPerRadian(float unitsPerRadian);
        void setDirection(int8_t dir);
        void updateData();
		void updatePosition();
        float relativePosition();
        float absolutePosition();
		float sampledVelocity();
        void reset();
};

#endif