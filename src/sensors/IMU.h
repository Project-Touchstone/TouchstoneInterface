/*
  IMU.h - Recieves and interprets IMU sensor data
  Created by Carson G. Ray
*/

#ifndef IMU_h
#define IMU_h

//External imports
#include <math.h>
#include <mutex>
#include <Eigen/Dense>
#include <stdint.h>

#define DPS_TO_RADS 0.017453293F

using namespace Eigen;

class IMU {
    public:
        enum AccelRange {
            ACCELRANGE_2G,
            ACCELRANGE_4G,
            ACCELRANGE_8G,
            ACCELRANGE_16G
        };
        enum GyroRange {
            GYRORANGE_250DPS,
            GYRORANGE_500DPS,
            GYRORANGE_1000DPS,
            GYRORANGE_2000DPS
        };

        void setRanges(AccelRange accelRange, GyroRange gyroRange);
        void updateAccelData(int16_t x, int16_t y, int16_t z);
        void updateGyroData(int16_t x, int16_t y, int16_t z);
    private:
        //Sensor range settings
        AccelRange accelRange = ACCELRANGE_2G;
        GyroRange gyroRange = GYRORANGE_250DPS;

        //Acceleromater data
        Vector3d accelData;
		//Gyroscope data
        Vector3d gyroData;

        //Mutex
		std::mutex mutex;
    
};

#endif