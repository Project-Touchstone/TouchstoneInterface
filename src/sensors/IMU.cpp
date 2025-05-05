/**
 * IMU.cpp - Recieves and interprets IMU sensor data
 * Created by Carson G. Ray
 */

#include "IMU.h"

using namespace std::chrono;
using namespace Eigen;
using namespace Utils;

IMU::IMU() {
    reset();
}

void IMU::setRanges(AccelRange accelRange, GyroRange gyroRange) {
	this->accelRange = accelRange;
	this->gyroRange = gyroRange;
}

void IMU::setOrientationOffset(Quaterniond offset) {
	mutex.lock();
	orientationOffset = offset;
	mutex.unlock();
    // Offsets initial orientation
    orientation = orientationOffset;
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
Quaterniond IMU::getOrientation() {
	mutex.lock();
	Quaterniond qOrientation = orientation;
	Quaterniond qOffset = orientationOffset;
	mutex.unlock();
    // Undoes initial offset
	return (qOrientation * qOffset.conjugate()).normalized();
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
    // Resets orientation to initial offset
    orientation = orientationOffset;

	// Resets error covariance
    P = Matrix3d::Identity();

    // Resets last sample time
	sampleTime = high_resolution_clock::now();
}

void IMU::updateOrientation() {
    // Get current accelerometer and gyroscope data
    Vector3d accel = getAccelData().normalized();
    Vector3d gyro = getGyroData();

    // Extract current yaw value from the orientation quaternion
    double initialYaw = atan2(2.0 * (orientation.w() * orientation.z() + orientation.x() * orientation.y()),
        1.0 - 2.0 * (orientation.y() * orientation.y() + orientation.z() * orientation.z()));

    // Time step
    mutex.lock();
    high_resolution_clock::time_point end = high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - sampleTime); // Use microseconds
    stepTime = duration.count() / 1000000.;
    sampleTime = end;
    mutex.unlock();

    // Step 1: Predict orientation using gyroscope data
    Vector3d delta = gyro * stepTime * 0.5;
    Quaterniond qDelta(0, delta.x(), delta.y(), delta.z());
	orientation = qAdd(orientation, qDelta * orientation).normalized();

    // Predict error covariance
    Matrix3d F = Matrix3d::Identity(); // State transition matrix
    F += skewSymmetric(gyro) * stepTime; // Incorporate angular velocity dynamics
    P = F * P * F.transpose() + Q;

    // Step 2: Update orientation using accelerometer data
    // Compute the expected gravity vector in the current orientation
	Vector3d expectedGravity = qRotate(orientation.conjugate(), Vector3d(0, 0, 1));

    // Compute the Kalman gain
    Matrix3d S = P + R;
    Matrix3d K = P* S.inverse();
    // Update error covariance
    P = (Matrix3d::Identity() - K) * P;

    // Gets filtered gravity direction vector
	Vector3d correctedGravity = expectedGravity + K * (accel - expectedGravity);

    // Update orientation
	Quaterniond qCorrection = Quaterniond::FromTwoVectors(correctedGravity, expectedGravity);
    orientation = (orientation * qCorrection).normalized();

    // Extract new yaw value after orientation update
    double updatedYaw = atan2(2.0 * (orientation.w() * orientation.z() + orientation.x() * orientation.y()),
        1.0 - 2.0 * (orientation.y() * orientation.y() + orientation.z() * orientation.z()));
    
    // Calculate the yaw correction needed to restore the original yaw
    double yawCorrectionAngle = initialYaw - updatedYaw;
    Quaterniond yawCorrection = Quaterniond(AngleAxisd(yawCorrectionAngle, Vector3d(0, 0, 1)));

    // Apply the yaw correction to maintain constant yaw
    orientation = (yawCorrection * orientation).normalized();
}

