/**
 * IMU.cpp - Recieves and interprets IMU sensor data
 * Created by Carson G. Ray
 */

#include "IMU.h"

using namespace std::chrono;

IMU::IMU() {
    reset();

    // Set process noise covariance (tune these values as needed)
    Q = Matrix3d::Identity() * 10;

    // Set measurement noise covariance (tune these values as needed)
    R = Matrix3d::Identity() * 0.001;

	// Initialize gyro offset and scale
	gyroOffset = Vector3d::Zero();
	accelScale = 1.0;

	// Initialize gyro sum and accel sum
	gyroSum = Vector3d::Zero();
	accelSum = 0.0;
}

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

	// Accumulates data for calibration
	if (inCalibrationMode) {
		accelSum += newAccel.norm();
		accelSamples++;
		if (accelSamples >= calibrationSamples) {
			// Sets scale factor to normalize to gravity
            accelScale = accelSum / accelSamples;
			accelCalibrated = true;
            if (isCalibrated()) {
                inCalibrationMode = false;
            }
		}
	}
}

void IMU::updateGyroData(int16_t x, int16_t y, int16_t z) {
   double gyro_scale = 1;
    if (gyroRange == GYRORANGE_250DPS)
        gyro_scale = 131 * 180 / EIGEN_PI;
    if (gyroRange == GYRORANGE_500DPS)
        gyro_scale = 65.5 * 180 / EIGEN_PI;
    if (gyroRange == GYRORANGE_1000DPS)
        gyro_scale = 32.8 * 180 / EIGEN_PI;
    if (gyroRange == GYRORANGE_2000DPS)
        gyro_scale = 16.4 * 180 / EIGEN_PI;

    Vector3d newGyro;
    newGyro << static_cast<double>(x) / gyro_scale,
        static_cast<double>(y) / gyro_scale,
        static_cast<double>(z) / gyro_scale;
    mutex.lock();
    gyroData = newGyro;
    mutex.unlock();

	// Accumulates data for calibration
	if (inCalibrationMode) {
		gyroSum += newGyro;
        gyroSamples++;
		if (gyroSamples >= calibrationSamples) {
			// Sets offset to average gyro data
			gyroOffset = gyroSum / gyroSamples;
			gyroCalibrated = true;
            if (isCalibrated()) {
                inCalibrationMode = false;
            }
		}
	}
}

Vector3d IMU::getGyroData() {
	mutex.lock();
	Vector3d gyro = gyroData;
	mutex.unlock();
	return gyro-gyroOffset;
}
Vector3d IMU::getAccelData() {
	mutex.lock();
	Vector3d accel = accelData;
	mutex.unlock();
	return accel/accelScale;
}
Vector3d IMU::getOrientation() {
	mutex.lock();
	Vector3d orientationVec = orientation.vec();
	mutex.unlock();
	return orientationVec;
}

bool IMU::isCalibrated() {
	return accelCalibrated && gyroCalibrated;
}

void IMU::calibrate() {
    // Initialize gyro sum and accel sum
    gyroSum = Vector3d::Zero();
    accelSum = 0.0;
    inCalibrationMode = true;
}

void IMU::reset() {
    // Resets orientation to identity quaternion
    orientation = Quaterniond::Identity();

	// Resets error covariance
    P = Matrix3d::Identity();

    // Resets last sample time
	sampleTime = high_resolution_clock::now();
}

void IMU::updateOrientation() {
    // Get current accelerometer and gyroscope data
    Vector3d accel = getAccelData().normalized();
    Vector3d gyro = getGyroData();

    // Time step
    mutex.lock();
    high_resolution_clock::time_point end = high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - sampleTime); // Use microseconds
    double dt = duration.count() / 1000000.;
    sampleTime = end;
    mutex.unlock();

    // Step 1: Predict orientation using gyroscope data
    Vector3d omega = gyro * dt; // Angular velocity * time step
    Quaterniond deltaOrientation(1, 0.5 * omega.x(), 0.5 * omega.y(), 0.5 * omega.z());
    orientation = (orientation * deltaOrientation).normalized();

    // Predict error covariance
    P = P + Q;

    // Step 2: Update orientation using accelerometer data
    // Compute the expected gravity vector in the current orientation
    Vector3d expectedGravity = orientation * Vector3d(0, 0, -1);

    // Compute the measurement residual
    Vector3d y = accel - expectedGravity;

    // Compute the Kalman gain
    Matrix3d S = P + R;
    Matrix3d K = P * S.inverse();

    // Update orientation
    Vector3d correction = K * y;
    Quaterniond correctionQuat(1, 0.5 * correction.x(), 0.5 * correction.y(), 0.5 * correction.z());
    orientation = (orientation * correctionQuat).normalized();

    // Update error covariance
    P = (Matrix3d::Identity() - K) * P;
}

