/**
 * DRIFTMotor.cpp - Dynamic resistance integrated force-feedback and tracking motor
 * Created by Carson G. Ray
*/

#include "DRIFTMotor.h"

/// @brief Default constructor
DRIFTMotor::DRIFTMotor() {
	//Dynamic memory allocation for spinlock
	spinlock = (portMUX_TYPE*) malloc(sizeof(portMUX_TYPE));
	// Initialize the spinlock dynamically
	portMUX_INITIALIZE(spinlock);
}

/// @brief Assigns servo and encoders to motor
/// @param servoChannel ServoController channel
/// @param encoderPort0 BusChain port for servo encoder
/// @param encoderPort1 BusChain port for spool encoder
/// @return -1 (successful), >-1 (encoder port where error occured)
int16_t DRIFTMotor::attach(uint8_t servoChannel, uint8_t encoderPort0, uint8_t encoderPort1) {
  	this->servoChannel = servoChannel;

	for (uint8_t i = 0; i < 2; i++) {
		//Assigns encoder ports
		uint8_t port;
		switch(i) {
			case 0:
				port = encoderPort0;
				break;
			case 1:
				port = encoderPort1;
				break;
		}
		encoderPorts[i] = port;

		//Opens BusChain channel and connects to encoder
		if (BusChain::selectPort(port) != 0) {
			return port;
		}
		if (!encoders[i].begin()) {
			return port;
		}
		BusChain::release();

		//Sets encoder direction
		encoders[i].setDirection(encoderDirs[i]);
	}
	return -1;
}

/// @brief Reads magnetic sensor data
void DRIFTMotor::updateSensor(uint8_t encoder) {
	BusChain::selectPort(encoderPorts[encoder]);
	encoders[encoder].updateData();
	BusChain::release();
}

/// @brief Interpolates encoder position
void DRIFTMotor::updateEncoder(uint8_t encoder) {
	encoders[encoder].updatePosition();
	if ((encoder == 1) && (getMode() == HOMING) && (getEncoderPos(1) < homePos)) {
		taskENTER_CRITICAL(spinlock);
		homePos = getEncoderPos(1);
		taskEXIT_CRITICAL(spinlock);
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
	taskENTER_CRITICAL(spinlock);
	for (uint8_t i = 0; i < 2; i++) {
		velocities[i] = encoders[i].sampledVelocity();
	}
	taskEXIT_CRITICAL(spinlock);
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
	//PID cannot be updated during calibration
	Mode currMode = getMode();
	if (currMode != CALIBRATION) {
		float necessaryVel = 0;
		taskENTER_CRITICAL(spinlock);
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
		
		taskEXIT_CRITICAL(spinlock);
		
		//Sets power based on necessary velocity
		setPower(necessaryVel*velocityCorrelation);
	}
}

/// @brief Sets servo power
/// @param power + (unspooling), - (spooling)
void DRIFTMotor::setPower(float power) {
  	ServoController::setPower(servoChannel, power*servoDir);
}

/// @brief Sets motor force applied
/// @param force distance tortional spring is engaged
void DRIFTMotor::setForceTarget(float force) {
  setMode(FORCE);
  taskENTER_CRITICAL(spinlock);
  if (force > 0) {
    separationTarget = spoolOffset + force/unitsPerRadian;
  } else {
	//If force is zero, no need to be right on the cusp of the tortional spring
    separationTarget = minSep;
  }
  taskEXIT_CRITICAL(spinlock);
}

/// @brief Sets spool POSITION limit
/// @param target POSITION limit
void DRIFTMotor::setPositionLimit(float target) {
  setMode(POSITION);
  taskENTER_CRITICAL(spinlock);
  posLimit = target/unitsPerRadian+homePos;
  taskEXIT_CRITICAL(spinlock);
}

/// @brief Gets current mode
/// @return mode enum
DRIFTMotor::Mode DRIFTMotor::getMode() {
	taskENTER_CRITICAL(spinlock);
	Mode currMode = mode;
	taskEXIT_CRITICAL(spinlock);
  	return currMode;
}

/// @brief Sets mode
/// @param mode mode enum
void DRIFTMotor::setMode(Mode mode) {
	taskENTER_CRITICAL(spinlock);
  	this->mode = mode;
	taskEXIT_CRITICAL(spinlock);
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
	taskENTER_CRITICAL(spinlock);
	float home = homePos;
	taskEXIT_CRITICAL(spinlock);
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
	taskENTER_CRITICAL(spinlock);
	float vel = velocities[encoder];
	taskEXIT_CRITICAL(spinlock);
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