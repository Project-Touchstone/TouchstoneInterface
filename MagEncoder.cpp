/**
 * MagEncoder.h - Custom ring-based multi-magnet encoder implementation
 * Created by Carson G. Ray
 */

#include "MagEncoder.h"

/// @brief Default constructor
MagEncoder::MagEncoder() {
	//Dynamic memory allocation for spinlock
	spinlock = (portMUX_TYPE*) malloc(sizeof(portMUX_TYPE));
	// Initialize the spinlock dynamically
	portMUX_INITIALIZE(spinlock);
}

/// @brief Initializes communication with magnetic sensor
/// @return true (successful), false (error)
bool MagEncoder::begin() {
	magSensor.begin();
	Wire.setTimeout(1000);
	Wire.setClock(1000000);
	//Sensor loads in new data after read command from master
	bool ret = magSensor.setAccessMode(magSensor.MASTERCONTROLLEDMODE);
	//No temperature data is necessary
	magSensor.disableTemp();

	//Sets initial sample time
	sampleStart = micros();

	//Initializes data update mutex
	mutex = xSemaphoreCreateMutex();

	return !ret;
}

/// @brief Sets encoder direction
/// @param dir 1 (forwards), -1 (backwards)
void MagEncoder::setDirection(int8_t dir) {
	taskENTER_CRITICAL(spinlock);
  	this->dir = dir;
	taskEXIT_CRITICAL(spinlock);
}

/// @brief Reads sensor data
void MagEncoder::updateData() {
	magSensor.updateData();
	xSemaphoreTake(mutex, portMAX_DELAY);
	sensorData[0] = magSensor.getY();
	sensorData[1] = magSensor.getZ();
	xSemaphoreGive(mutex);
}

/// @brief Updates position
void MagEncoder::updatePosition() {
	xSemaphoreTake(mutex, portMAX_DELAY);
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
    	yVals[0] = sensorData[0]/amplitudes[0];
		yVals[1] = sensorData[1]/amplitudes[1];
		xSemaphoreGive(mutex);
		for (uint8_t i = 0; i < 2; i++) {
			//Gets angle (-pi to pi) based on sinusoidal approximation
			angles[i][0] = asin(yVals[i]);

			//Gets alternate angle solution with the same sine value
			if (angles[i][0] >= 0) {
				angles[i][1] = PI-angles[i][0];
			} else {
				angles[i][1] = -PI-angles[i][0];
			}

			//Applies phase shifts to axes and ensures angles stay within range
			for (uint8_t j = 0; j < 2; j++) {
				angles[i][j] += phases[i];
				if (angles[i][j] > PI) {
					angles[i][j] -= 2*PI;
				} else if (angles[i][j] < -PI) {
					angles[i][j] += 2*PI;
				}
			}
      	}

		//Finds the axis with the maximum spread between solutions
      	float maxSpread;
      	uint8_t maxLoc;
      	for (uint8_t i = 0; i < 2; i++) {
			float spread = abs(angles[i][0] - angles[i][1]);
			//If spread is greater than PI, the wraparound distance is smaller
			if (spread > PI) {
				spread = 2*PI - spread;
			}
			if ((i == 0) || spread > maxSpread) {
				maxSpread = spread;
				maxLoc = i;
			}
      	}

		//Finds solution on target axis with the minimum distance from the last calculated angle
		float minDist;
		float finalAngle;
		for (uint8_t i = 0; i < 2; i++) {
			float dist = abs(angles[maxLoc][i] - prevAngle);
			//If distance is greater than PI, the wraparound distance is smaller
			if (dist > PI) {
				dist = 2*PI - dist;
			}
			if ((i == 0) || dist < minDist) {
				minDist = dist;
				finalAngle = angles[maxLoc][i];
			}
		}

		//Gets change in calculated angle
		float diff = (finalAngle-prevAngle)*dir;
		//Updates previous angle
		prevAngle = finalAngle;

		//Detects wraparound
		if (diff > PI) {
			diff -= 2*PI;
		} else if (diff < -PI) {
			diff += 2*PI;
		}

		//Updates position
		taskENTER_CRITICAL(spinlock);
		position += diff;
		taskEXIT_CRITICAL(spinlock);
    } else {
		xSemaphoreGive(mutex);
	}
}

/// @brief Gets position relative to last reset
/// @return position in user units
float MagEncoder::relativePosition() {
	taskENTER_CRITICAL(spinlock);
  	float relPos = (position-offset);
	taskEXIT_CRITICAL(spinlock);
	return relPos;
}

/// @brief Gets position relative to start of program
/// @return position in user units
float MagEncoder::absolutePosition() {
	taskENTER_CRITICAL(spinlock);
  	float absPos = position;
	taskEXIT_CRITICAL(spinlock);
	return absPos;
}

/// @brief Gets average velocity in the time period since the last call
/// @return velocity in units per second
float MagEncoder::sampledVelocity() {
	taskENTER_CRITICAL(spinlock);
	float velocity = (position-lastPosition)/((micros() - sampleStart)/1000000.);
	lastPosition = position;
	taskEXIT_CRITICAL(spinlock);
	sampleStart = micros();

	return velocity;
}

/// @brief Updates encoder data and resets relative position to zero
void MagEncoder::reset() {
	taskENTER_CRITICAL(spinlock);
  	offset = position;
	lastPosition = position;
	sampleStart = micros();
	taskEXIT_CRITICAL(spinlock);
}