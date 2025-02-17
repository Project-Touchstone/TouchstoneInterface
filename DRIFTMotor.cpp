/**
 * DRIFTMotor.cpp - Dynamic resistance integrated force-feedback and tracking motor
 * Created by Carson G. Ray
*/

#include "DRIFTMotor.h"

/// @brief Assigns motor and encoder ids
/// @param motorID motor ID
/// @param servoSensorID sensor ID for servo encoder
/// @param spoolSensorID sensor ID for spool encoder
void DRIFTMotor::attach(uint8_t motorID, uint8_t servoSensorID, uint8_t spoolSensorID) {
  	this->motorID = motorID;

	for (uint8_t i = 0; i < 2; i++) {
		//Attaches sensor ids to encoders
		uint8_t sensorID;
		switch(i) {
			case 0:
				sensorID = servoSensorID;
				break;
			case 1:
				sensorID = spoolSensorID;
				break;
		}
		encoders[i].begin(sensorID);
		//Sets encoder direction
		encoders[i].setDirection(encoderDirs[i]);
	}
}

/// @brief Resets both motor encoders
void DRIFTMotor::resetEncoders() {
	for (uint8_t i = 0; i < 2; i++) {
		encoders[i].reset();
	}
}

void DRIFTMotor::sampleVelocity() {
	//Updates sampled encoder velocities
	for (uint8_t i = 0; i < 2; i++) {
		velocities[i] = encoders[i].sampledVelocity();
	}
}

/// @brief Updates servo model predictive control
void DRIFTMotor::updateMPC() {
	//Samples velocity and predicts position of spool
	sampleVelocity();
	updateMPCLocal(getPredEncoderPos(1));
}

/// @brief Updates servo model predictive control
/// @param predictedPos predicted spool position in external units
void DRIFTMotor::updateMPC(float predictedPos) {
	updateMPCLocal((predictedPos/unitsPerRadian) + homePos);
}

/// @brief Updates servo model predictive control
/// @param predictedPos predicted spool position
void DRIFTMotor::updateMPCLocal(float predictedPos) {
	Mode currMode = getMode();
	// Updates homing position
	if ((currMode == HOMING) && (getEncoderPos(1) < homePos)) {
		homePos = getEncoderPos(1);
	}
	//PID cannot be updated during manual mode
	if (currMode != MANUAL) {
		float necessaryVel = 0;
		mutex.lock();
		if (currMode == POSITION) {
			if (predictedPos < posLimit) {
				//Separation target is set to enforce desired POSITION
				separationTarget = predictedPos - (posLimit-spoolOffset);

				if (separationTarget < minSep) {
					//A minimum separation prevents string from becoming slack
					separationTarget = minSep;
				}
			} else {
				necessaryVel = (posLimit - predictedPos)/(horizonTime/1000000.);
			}
		}
		if (currMode != POSITION || (currMode == POSITION && predictedPos < posLimit)) {
			//Gets necessary spool velocity to reach separation target from predicted servo position
			necessaryVel = ((predictedPos-separationTarget)-getEncoderPos(0))/(horizonTime/1000000.);
		}
		
		mutex.unlock();
		
		//Sets power based on necessary velocity
		this->power = necessaryVel*velocityCorrelation*motorDir;
	}
}

/// @brief Sets motor power
/// @param power + (unspooling), - (spooling)
void DRIFTMotor::setPower(float power) {
	setMode(MANUAL);
  	this->power = power*motorDir;
}

/// @brief Gets motor power
float DRIFTMotor::getPower() {
	return power;
}

/// @brief Sets motor force applied
/// @param force distance tortional spring is engaged
void DRIFTMotor::setForceTarget(float force) {
  setMode(FORCE);
  mutex.lock();
  if (force > 0) {
    separationTarget = spoolOffset + force/unitsPerRadian;
  } else {
	//If force is zero, no need to be right on the cusp of the tortional spring
    separationTarget = minSep;
  }
  mutex.unlock();
}

/// @brief Sets spool POSITION limit
/// @param target POSITION limit
void DRIFTMotor::setPositionLimit(float target) {
  setMode(POSITION);
  mutex.lock();
  posLimit = target/unitsPerRadian+homePos;
  mutex.unlock();
}

/// @brief Gets current mode
/// @return mode enum
DRIFTMotor::Mode DRIFTMotor::getMode() {
	mutex.lock();
	Mode currMode = mode;
	mutex.unlock();
  	return currMode;
}

/// @brief Sets mode
/// @param mode mode enum
void DRIFTMotor::setMode(Mode mode) {
	mutex.lock();
  	this->mode = mode;
	mutex.unlock();
}

void DRIFTMotor::beginHoming() {
	setForceTarget(0);
	setMode(HOMING);
}

void DRIFTMotor::endHoming() {
	setMode(FORCE);
}

/// @brief Gets the position of an encoder
/// @param encoder 0 (servo), 1 (spool)
/// @return encoder position
float DRIFTMotor::getEncoderPos(uint8_t encoder) {
	return encoders[encoder].relativePosition();
}
/// @brief Gets the position of the motor after homing
/// @return position
float DRIFTMotor::getPosition() {
	mutex.lock();
	float home = homePos;
	mutex.unlock();
  return (getEncoderPos(1) - home)*unitsPerRadian;
}

/// @brief Gets next predicted position of spool after horizon time
/// @return position
float DRIFTMotor::getPredEncoderPos(uint8_t encoder) {
	return getEncoderPos(encoder) + getEncoderVel(encoder)*horizonTime/1000000.;
}

float DRIFTMotor::getPredictedPos() {
	return (getPredEncoderPos(1) - homePos)*unitsPerRadian;
}

/// @brief Gets the velocity of an encoder
/// @param encoder 0 (servo encoder), 1 (spool encoder)
/// @return velocity in units per second
float DRIFTMotor::getEncoderVel(uint8_t encoder) {
	mutex.lock();
	float vel = velocities[encoder];
	mutex.unlock();
  	return velocities[encoder];
}
/// @brief Gets the velocity of the motor spool
/// @return velocity
float DRIFTMotor::getVelocity() {
	return getEncoderVel(1)*unitsPerRadian;
}

/// @brief Gets separation between spool and servo encoders
/// @return separation
float DRIFTMotor::getSeparation() {
  return (getEncoderPos(1) - getEncoderPos(0))*unitsPerRadian;
}

uint32_t DRIFTMotor::getHorizonTime() {
	return horizonTime;
}