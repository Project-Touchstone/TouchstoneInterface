/**
 * DRIFTMotor.cpp - Dynamic resistance integrated force-feedback and tracking motor
 * Created by Carson G. Ray
*/

#include "DRIFTMotor.h"

const double DRIFTMotor::unitsPerRadian = 24.5 / 12;
const double DRIFTMotor::springConstant = 100.; //N*mm/rad
const double DRIFTMotor::spoolOffset = 15;
const uint32_t DRIFTMotor::horizonTime = 20000;

/// @brief Links to encoder objects
/// @param servoEncoder servo encoder pointer
/// @param spoolEncoder spool encoder pointer
void DRIFTMotor::attach(MagEncoder* servoEncoder, MagEncoder* spoolEncoder) {
	encoders[0] = servoEncoder;
	encoders[1] = spoolEncoder;
	for (uint8_t i = 0; i < 2; i++) {
		encoders[i]->reset();
		//Sets encoder direction
		encoders[i]->setDirection(encoderDirs[i]);
	}
}

/// @brief Resets both motor encoders
void DRIFTMotor::resetEncoders() {
	for (uint8_t i = 0; i < 2; i++) {
		encoders[i]->reset();
	}
}

void DRIFTMotor::sampleVelocity() {
	//Updates sampled encoder velocities
	for (uint8_t i = 0; i < 2; i++) {
		velocities[i] = encoders[i]->sampledVelocity();
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
void DRIFTMotor::updateMPC(double predictedPos) {
	updateMPCLocal((predictedPos/unitsPerRadian) + homePos);
}

/// @brief Updates servo model predictive control
/// @param predictedPos predicted spool position
void DRIFTMotor::updateMPCLocal(double predictedPos) {
	Mode currMode = getMode();
	// Updates homing position
	if (isHoming() && (getEncoderPos(1) < homePos)) {
		homePos = getEncoderPos(1);
	}
	//PID cannot be updated during manual mode
	if (currMode != MANUAL) {
		double necessaryVel = 0;
		{
			std::lock_guard<std::mutex> lock(dataMutex);
			if (currMode == POSITION) {
				if (predictedPos < posLimit) {
					//Separation target is set to enforce desired POSITION
					separationTarget = predictedPos - (posLimit - spoolOffset);

					if (separationTarget < minSep) {
						//A minimum separation prevents string from becoming slack
						separationTarget = minSep;
					}
				}
				else {
					necessaryVel = (posLimit - predictedPos) / (horizonTime / 1000000.);
				}
			}
			if (currMode != POSITION || (currMode == POSITION && predictedPos < posLimit)) {
				// Ensures separation is not too small
				if (predictedPos - separationTarget > getEncoderPos(1) - minSep) {
					predictedPos = getEncoderPos(1) - minSep + separationTarget;
				}
				//Gets necessary spool velocity to reach separation target from predicted servo position
				necessaryVel = ((predictedPos - separationTarget) - getEncoderPos(0)) / (horizonTime / 1000000.);
			}
		}
		
		//Sets power based on necessary velocity
		setPowerLocal(necessaryVel*velocityCorrelation);
	}
}

void DRIFTMotor::setPowerLocal(double power) {
	if (power > 1) {
		power = 1;
	}
	else if (power < -1) {
		power = -1;
	}
	std::lock_guard<std::mutex> lock(dataMutex);
	this->power = power * motorDir;
}

/// @brief Sets motor power
/// @param power + (unspooling), - (spooling)
void DRIFTMotor::setPower(double power) {
	setMode(MANUAL);
	setPowerLocal(power);
}

/// @brief Gets motor power
double DRIFTMotor::getPower() {
	std::lock_guard<std::mutex> lock(dataMutex);
	return power;
}

/// @brief Sets motor force applied
/// @param desired force in N
void DRIFTMotor::setForceTarget(double force) {
	setMode(FORCE);
	std::lock_guard<std::mutex> lock(dataMutex);
	if (force < 0) {
		//Converts force in Newtons to necessary radians to turn
		separationTarget = spoolOffset - (force * unitsPerRadian / springConstant);
	} else {
		//If force is zero, no need to be right on the cusp of the tortional spring
		separationTarget = minSep;
	}
}

/// @brief Sets spool POSITION limit
/// @param target POSITION limit
void DRIFTMotor::setPositionLimit(double target) {
	  setMode(POSITION);
	  std::lock_guard<std::mutex> lock(dataMutex);
	  posLimit = target/unitsPerRadian+homePos;
}

/// @brief Gets current mode
/// @return mode enum
DRIFTMotor::Mode DRIFTMotor::getMode() {
	std::lock_guard<std::mutex> lock(dataMutex);
  	return mode;
}

/// @brief Sets mode
/// @param mode mode enum
void DRIFTMotor::setMode(Mode mode) {
	std::lock_guard<std::mutex> lock(dataMutex);
  	this->mode = mode;
}

void DRIFTMotor::beginHoming() {
	std::lock_guard<std::mutex> lock(dataMutex);
	homing = true;
}

void DRIFTMotor::endHoming() {
	std::lock_guard<std::mutex> lock(dataMutex);
	homing = false;
}

bool DRIFTMotor::isHoming() {
	std::lock_guard<std::mutex> lock(dataMutex);
	return homing;
}

/// @brief Gets the position of an encoder
/// @param encoder 0 (servo), 1 (spool)
/// @return encoder position
double DRIFTMotor::getEncoderPos(uint8_t encoder) {
	return encoders[encoder]->relativePosition();
}
/// @brief Gets the position of the motor after homing
/// @return position
double DRIFTMotor::getPosition() {
	std::lock_guard<std::mutex> lock(dataMutex);
  return (getEncoderPos(1) - homePos)*unitsPerRadian;
}

/// @brief Gets next predicted position of spool after horizon time
/// @return position
double DRIFTMotor::getPredEncoderPos(uint8_t encoder) {
	return getEncoderPos(encoder) + getEncoderVel(encoder)*horizonTime/1000000.;
}

double DRIFTMotor::getPredictedPos() {
	return (getPredEncoderPos(1) - homePos)*unitsPerRadian;
}

/// @brief Gets the velocity of an encoder
/// @param encoder 0 (servo encoder), 1 (spool encoder)
/// @return velocity in units per second
double DRIFTMotor::getEncoderVel(uint8_t encoder) {
	std::lock_guard<std::mutex> lock(dataMutex);
  	return velocities[encoder];
}
/// @brief Gets the velocity of the motor spool
/// @return velocity
double DRIFTMotor::getVelocity() {
	return getEncoderVel(1)*unitsPerRadian;
}

/// @brief Gets separation between spool and servo encoders
/// @return separation
double DRIFTMotor::getSeparation() {
  return (getEncoderPos(1) - getEncoderPos(0))*unitsPerRadian;
}

uint32_t DRIFTMotor::getHorizonTime() {
	return horizonTime;
}

double DRIFTMotor::getSpoolOffset() {
	return spoolOffset*unitsPerRadian;
}