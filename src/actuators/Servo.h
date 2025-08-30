/**
 * Servo.h - Servo wrapper class
 * Created by Carson G. Ray
*/

#ifndef SERVO_H
#define SERVO_H

#include <Eigen/Dense>

class Servo {
	public:
		void writePower(float power);
		void writeAngle(float angle);

		float getSignal();
	private:
		float signal = 0;
};

#endif