/**
 * HydraFOCMotor.cpp - Force-feedback and tracking motor
 * Created by Carson G. Ray
*/

#include "HydraFOCMotor.h"

double HydraFOCMotor::rotorRadius = 0.005;

/// @brief Links to encoder object
/// @param motor encoder
void HydraFOCMotor::attach(MagEncoder* encoder) {
	this->encoder = encoder;
	encoder->setDirection(1);
	encoder->reset();
}

/// @brief Resets both motor encoders
void HydraFOCMotor::resetEncoder() {
	encoder->reset();
}

/// @brief Updates motor data
void HydraFOCMotor::update() {
	// Updates homing position
	if (isHoming() && (getEncoderPos() < homePos)) {
		homePos = getEncoderPos();
	}
}

/// @brief Sets motor force target
/// @param force (N) + (unspooling), - (spooling)
void HydraFOCMotor::setForceTarget(double force) {
	setMode(FORCE);
	torqueTarget = motorDir*force*getSpoolRadius();
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
	velTarget = motorDir * velocity / getSpoolRadius();
}

/// @brief Gets motor angular velocity target (rad/s)
double HydraFOCMotor::getVelocityTarget() {
	std::lock_guard<std::mutex> lock(dataMutex);
	return velTarget;
}

/// @brief Sets motor direction
/// @param motorDir 1 (regular), -1 (inverted)
void HydraFOCMotor::setMotorDir(int8_t motorDir) {
	std::lock_guard<std::mutex> lock(dataMutex);
	this->motorDir = motorDir;
}

void HydraFOCMotor::setEncoderDir(int8_t encoderDir) {
	encoder->setDirection(encoderDir);
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
  return motorDir * (getEncoderPos() - homePos)*getSpoolRadius();
}

double HydraFOCMotor::getSpoolRadius() {
	return rotorRadius;
}