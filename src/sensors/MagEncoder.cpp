/**
 * MagEncoder.h - Custom ring-based multi-magnet encoder implementation
 * Created by Carson G. Ray
 */

#include "MagEncoder.h"

using namespace std::chrono;

MagEncoder::MagEncoder() {}

/// @brief Sets encoder direction
/// @param dir 1 (forwards), -1 (backwards)
void MagEncoder::setDirection(int8_t dir) {
    std::lock_guard<std::mutex> lock(dataMutex);
    this->dir = dir;
}

/// @brief Updates external sensor data and calculates position
void MagEncoder::updateData(uint16_t rawAngle) {
	// Gets current angle
	double currAngle = rawAngle * rawToRadians;
	//Gets change in calculated angle
	double diff = (currAngle - prevAngle) * dir;
	//Updates previous angle
	prevAngle = currAngle;

	//Detects wraparound
	if (diff > EIGEN_PI) {
		diff -= 2 * EIGEN_PI;
	}
	else if (diff < -EIGEN_PI) {
		diff += 2 * EIGEN_PI;
	}

	//Updates position
	std::lock_guard<std::mutex> lock(dataMutex);
	position += diff;
}

/// @brief Gets position relative to last reset
/// @return position in user units
double MagEncoder::relativePosition() {
	std::lock_guard<std::mutex> lock(dataMutex);
	return position - offset;
}

/// @brief Gets position relative to start of program
/// @return position in user units
double MagEncoder::absolutePosition() {
	std::lock_guard<std::mutex> lock(dataMutex);
	return position;
}

/// @brief Updates encoder data and resets relative position to zero
void MagEncoder::reset() {
	timer.reset();
	std::lock_guard<std::mutex> lock(dataMutex);
  	offset = position;
	lastPosition = position;
}