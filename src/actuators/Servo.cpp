/**
 * Servo.cpp - Servo wrapper class
 * Created by Carson G. Ray
*/

#include "Servo.h"

void Servo::writePower(float power) {
	signal = power;
}

void Servo::writeAngle(float angle) {
	signal = (angle - (float)EIGEN_PI/2)/((float)EIGEN_PI/2);
}

float Servo::getSignal() {
	return signal;
}