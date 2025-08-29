/**
 * HydraFOCMotor.cpp - Force-feedback and tracking motor
 * Created by Carson G. Ray
*/

#include "HydraFOCMotor.h"

const double HydraFOCMotor::rotorRadius = 5;

/// @brief Links to encoder object
/// @param motor encoder
void HydraFOCMotor::attach(MagEncoder* encoder) {
	this->encoder = encoder;
	encoder->reset();
}

/// @brief Resets both motor encoders
void HydraFOCMotor::resetEncoder() {
	encoder->reset();
}

void HydraFOCMotor::sampleVelocity() {
	//Updates sampled encoder velocities
	velocity = encoder->sampledVelocity();
}

/// @brief Updates motor data
void HydraFOCMotor::update() {
	//Samples velocity
	sampleVelocity();
	// Updates homing position
	if (isHoming() && (getEncoderPos() < homePos)) {
		homePos = getEncoderPos();
	}
}

/// @brief Sets motor force target
/// @param force (N) + (unspooling), - (spooling)
void HydraFOCMotor::setForceTarget(double force) {
	setMode(FORCE);
	torqueTarget = force*getSpoolRadius();
}

/// @brief Gets motor torque target (Nm)
double HydraFOCMotor::getTorqueTarget() {
	std::lock_guard<std::mutex> lock(dataMutex);
	return torqueTarget;
}

/// @brief Sets motor velocity target
/// @param velocity (m/s) + (unspooling), - (spooling)
void HydraFOCMotor::setVelocityTarget(double velocity) {
	setMode(VELOCITY);
	velTarget = velocity / getSpoolRadius();
}

/// @brief Gets motor angular velocity target (rad/s)
double HydraFOCMotor::getOmegaTarget() {
	std::lock_guard<std::mutex> lock(dataMutex);
	return velTarget;
}

/// @brief Sets motor position target
/// @param position (mm) + (unspooling), - (spooling)
void HydraFOCMotor::setPositionTarget(double position) {
	setMode(POSITION);
	posTarget = position / getSpoolRadius();
}

/// @brief Gets motor position target (rad)
double HydraFOCMotor::getPositionTarget() {
	std::lock_guard<std::mutex> lock(dataMutex);
	return posTarget;
}

/// @brief Sets motor direction
/// @param motorDir 1 (regular), -1 (inverted)
void HydraFOCMotor::setMotorDir(int8_t motorDir) {
	std::lock_guard<std::mutex> lock(dataMutex);
	this->motorDir = motorDir;
}

/// @brief Gets current mode
/// @return mode enum
HydraFOCMotor::Mode HydraFOCMotor::getMode() {
	std::lock_guard<std::mutex> lock(dataMutex);
  	return mode;
}

/// @brief Sets mode
/// @param mode mode enum
void HydraFOCMotor::setMode(Mode mode) {
	std::lock_guard<std::mutex> lock(dataMutex);
  	this->mode = mode;
}

void HydraFOCMotor::beginHoming() {
	std::lock_guard<std::mutex> lock(dataMutex);
	homing = true;
}

void HydraFOCMotor::endHoming() {
	std::lock_guard<std::mutex> lock(dataMutex);
	homing = false;
}

bool HydraFOCMotor::isHoming() {
	std::lock_guard<std::mutex> lock(dataMutex);
	return homing;
}

/// @brief Gets the position of an encoder
/// @param encoder 0 (servo), 1 (spool)
/// @return encoder position
double HydraFOCMotor::getEncoderPos() {
	return encoder->relativePosition();
}
/// @brief Gets the position of the motor after homing
/// @return position
double HydraFOCMotor::getPosition() {
	std::lock_guard<std::mutex> lock(dataMutex);
  return (getEncoderPos() - homePos)*getSpoolRadius();
}

/// @brief Gets the velocity of an encoder
/// @param encoder 0 (servo encoder), 1 (spool encoder)
/// @return velocity in units per second
double HydraFOCMotor::getEncoderVel() {
	std::lock_guard<std::mutex> lock(dataMutex);
  	return velocity;
}
/// @brief Gets the velocity of the motor spool
/// @return velocity
double HydraFOCMotor::getVelocity() {
	return getEncoderVel()*getSpoolRadius();
}

double HydraFOCMotor::getSpoolRadius() {
	return rotorRadius;
}