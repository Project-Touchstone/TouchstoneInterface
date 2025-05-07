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
#include <chrono>
#include <iostream>

//Local imports
#include "../utils/Utils.h"

using namespace Eigen;
using namespace std::chrono;

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
        IMU();
        void setRanges(AccelRange accelRange, GyroRange gyroRange);
        void setOrientationOffset(Quaterniond offset);
        void updateAccelData(int16_t x, int16_t y, int16_t z);
        void updateGyroData(int16_t x, int16_t y, int16_t z);
        Vector3d getGyroData();
		Vector3d getAccelData();
        Quaterniond getOrientation();

		bool isCalibrated();
		void calibrate();
        void reset();
        void updateOrientation(double stepTime);
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

        // How many calibration samples to take
		const int calibrationSamples = 1000;

        //Whether IMU is in calibration mode
		bool inCalibrationMode = false;

		//Whether the IMU is calibrated
		bool accelCalibrated = false;
		bool gyroCalibrated = false;

        // Calibration parameters
		Vector3d gyroOffset = Vector3d::Zero(); // Gyroscope offset
		double accelScale = 1.0; // Accelerometer scale factor
		//Accumulates gyro data for calibration
		Vector3d gyroSum = Vector3d::Zero();
		//Accumulates accelerometer data for calibration
        double accelSum = 0;
        //Counts calibration samples
		int accelSamples = 0;
        int gyroSamples = 0;
        //Orientation offset based on IMU location
		Quaterniond orientationOffset = Quaterniond::Identity();

        // Kalman filter state
        high_resolution_clock::time_point sampleTime;
        Quaterniond orientation = Quaterniond::Identity(); // Current orientation as a quaternion
        Matrix3d P = Matrix3d::Identity();        // Error covariance matrix
        Matrix3d Q = Matrix3d::Identity() * 10; // Process noise covariance matrix
        Matrix3d R = Matrix3d::Identity() * 0.001; // Measurement noise covariance matrix
};

#endif