/**
 * MagEncoder.h - Custom ring-based multi-magnet encoder implementation
 * Created by Carson G. Ray
 */

#include "MagEncoder.h"

using namespace std::chrono;

/// @brief Sets encoder direction
/// @param dir 1 (forwards), -1 (backwards)
void MagEncoder::setDirection(int8_t dir) {
	mutex.lock();
  	this->dir = dir;
	mutex.unlock();
}

/// @brief Updates external sensor data and calculates position
void MagEncoder::updateData(double sensorData[2]) {
	for (int i = 0; i < 2; i++) {
		this->sensorData[i] = sensorData[i];
	}
	
	//If the value in a particular axis has greater magnitude, update maximum amplitude
	//Note: X-axis is not used becuase it does not change significantly
	if (abs(sensorData[0]) > amplitudes[0]) {
		amplitudes[0] = abs(sensorData[0]);
	}
	if (abs(sensorData[1]) > amplitudes[1]) {
		amplitudes[1] = abs(sensorData[1]);
	}

	//If all maximum amplitudes are nonzero
	if (abs(amplitudes[0]) > 0 && abs(amplitudes[1]) > 0) {
		//Normalizes axis values by maximum observed amplitude
		yVals[0] = sensorData[0] / amplitudes[0];
		yVals[1] = sensorData[1] / amplitudes[1];
		for (uint8_t i = 0; i < 2; i++) {
			//Gets angle (-PI to PI) based on sinusoidal approximation
			angles[i][0] = asin(yVals[i]);

			//Gets alternate angle solution with the same sine value
			if (angles[i][0] >= 0) {
				angles[i][1] = EIGEN_PI - angles[i][0];
			}
			else {
				angles[i][1] = -EIGEN_PI - angles[i][0];
			}

			//Applies phase shifts to axes and ensures angles stay within range
			for (uint8_t j = 0; j < 2; j++) {
				angles[i][j] += phases[i];
				if (angles[i][j] > EIGEN_PI) {
					angles[i][j] -= 2 * EIGEN_PI;
				}
				else if (angles[i][j] < -EIGEN_PI) {
					angles[i][j] += 2 * EIGEN_PI;
				}
			}
		}

		//Finds the axis with the maximum spread between solutions
		double maxSpread;
		uint8_t maxLoc;
		for (uint8_t i = 0; i < 2; i++) {
			double spread = abs(angles[i][0] - angles[i][1]);
			//If spread is greater than EIGEN_PI, the wraparound distance is smaller
			if (spread > EIGEN_PI) {
				spread = 2 * EIGEN_PI - spread;
			}
			if ((i == 0) || spread > maxSpread) {
				maxSpread = spread;
				maxLoc = i;
			}
		}

		//Finds solution on target axis with the minimum distance from the last calculated angle
		double minDist;
		double finalAngle;
		for (uint8_t i = 0; i < 2; i++) {
			double dist = abs(angles[maxLoc][i] - prevAngle);
			//If distance is greater than EIGEN_PI, the wraparound distance is smaller
			if (dist > EIGEN_PI) {
				dist = 2 * EIGEN_PI - dist;
			}
			if ((i == 0) || dist < minDist) {
				minDist = dist;
				finalAngle = angles[maxLoc][i];
			}
		}

		//Gets change in calculated angle
		double diff = (finalAngle - prevAngle) * dir;
		//Updates previous angle
		prevAngle = finalAngle;

		//Detects wraparound
		if (diff > EIGEN_PI) {
			diff -= 2 * EIGEN_PI;
		}
		else if (diff < -EIGEN_PI) {
			diff += 2 * EIGEN_PI;
		}

		//Updates position
		mutex.lock();
		position += diff;
		mutex.unlock();
	}
}

/// @brief Gets position relative to last reset
/// @return position in user units
double MagEncoder::relativePosition() {
	mutex.lock();
  	double relPos = (position-offset);
	mutex.unlock();
	return relPos;
}

/// @brief Gets position relative to start of program
/// @return position in user units
double MagEncoder::absolutePosition() {
	mutex.lock();
  	double absPos = position;
	mutex.unlock();
	return absPos;
}

/// @brief Gets average velocity in the time period since the last call
/// @return velocity in units per second
double MagEncoder::sampledVelocity() {
	mutex.lock();
	high_resolution_clock::time_point end = high_resolution_clock::now();
	auto duration = duration_cast<seconds>(end - sampleStart);
	double velocity = (position-lastPosition)/duration.count();
	lastPosition = position;
	mutex.unlock();
	sampleStart = high_resolution_clock::now();

	return velocity;
}

/// @brief Updates encoder data and resets relative position to zero
void MagEncoder::reset() {
	mutex.lock();
  	offset = position;
	lastPosition = position;
	sampleStart = high_resolution_clock::now();
	mutex.unlock();
}