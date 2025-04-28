/**
 * IMU.cpp - Recieves and interprets IMU sensor data
 * Created by Carson G. Ray
 */

#include "IMU.h"

void IMU::setRanges(AccelRange accelRange, GyroRange gyroRange) {
	this->accelRange = accelRange;
	this->gyroRange = gyroRange;
}

void IMU::updateAccelData(int16_t x, int16_t y, int16_t z) {
    float accel_scale = 1;
    if (accelRange == ACCELRANGE_16G)
        accel_scale = 2048;
    if (accelRange == ACCELRANGE_8G)
        accel_scale = 4096;
    if (accelRange == ACCELRANGE_4G)
        accel_scale = 8192;
    if (accelRange == ACCELRANGE_2G)
        accel_scale = 16384;

    Vector3d newAccel;
    newAccel << (double)(static_cast<float>(x) / accel_scale),
        (double)(static_cast<float>(y) / accel_scale),
        (double)(static_cast<float>(z) / accel_scale);
    mutex.lock();
    accelData = newAccel;
    mutex.unlock();
}

void IMU::updateGyroData(int16_t x, int16_t y, int16_t z) {
   float gyro_scale = 1;
    if (gyroRange == GYRORANGE_250DPS)
        gyro_scale = 131;
    if (gyroRange == GYRORANGE_500DPS)
        gyro_scale = 65.5;
    if (gyroRange == GYRORANGE_1000DPS)
        gyro_scale = 32.8;
    if (gyroRange == GYRORANGE_2000DPS)
        gyro_scale = 16.4;

    Vector3d newGyro;
    newGyro << (double)(static_cast<float>(x) / gyro_scale),
        (double)(static_cast<float>(y) / gyro_scale),
        (double)(static_cast<float>(z) / gyro_scale);
    mutex.lock();
    gyroData = newGyro;
    mutex.unlock();
}

