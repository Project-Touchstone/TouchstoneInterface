#include "MagTracker.h"

using namespace Eigen;
using namespace Utils;

void MagTracker::setSensorOrientation(Quaterniond sensorOrient) {
	this->sensorOrient = sensorOrient;
}

void MagTracker::storeRawData(const std::array<int16_t, 3>& data) {
    std::lock_guard<std::mutex> lock(mutex);
	for (uint8_t i = 0; i < 2; i++) {
		sensorData(i) = static_cast<double>(data[i]) * magSensorMultiplier;
	}
}

void MagTracker::updateData(Quaterniond magOrient) {
	//Transforms sensor data from sensor reference frame to magnet reference frame
	Vector3d Bfield = qRotate(sensorOrient * magOrient.conjugate(), sensorData);

	//Gets lateral and vertical components of magnetic field
	double Bc = sqrt(Bfield.x() + Bfield.y());
	double Bz = Bfield.z();

	//Calculates phi angle
	double phi = atan2(Bfield.y(), Bfield.x());

	//Gets old theta angle and associated sign change
	double prevTheta = acos(position.dot(Vector3d(0, 0, 1)) / position.norm());
	//Gets sign of cosine of angle
	int sign = -1;
	if (prevTheta < EIGEN_PI / 2) {
		sign = 1;
	}

	//Calculates theta angle
	double theta = 0;
	//Checks for special cases
	if (Bc == 0) {
		theta = 0;
	} else if (Bz == 0) {
		theta = acos(sign * pow(1 / 3, 0.5));
	}
	else {
		//Otherwise solves quadratic
		double a = 9 / pow(Bz, 2) + 9 / pow(Bc, 2);
		double b = -(6 / pow(Bz, 2) + 9 / pow(Bc, 2));
		double c = 1 / pow(Bz, 2);

		//Determines which side of the axis of symmetry solution is on
		int axisSide = -1;
		if (pow(cos(prevTheta), 2) > -b / (2 * a)) {
			axisSide = 1;
		}
		double root = (-b + axisSide * pow(pow(b, 2) - 4 * a * c, 0.5)) / (2 * a);
		theta = acos(sign * pow(root, 0.5));
	}

	//Calculates radius
	double radius = pow(3 / Bc * cos(theta) * sin(theta), 1 / 3);

	//Combines sphereical coordinates to get final relative position vector
	position.x() = radius * sin(theta) * cos(phi);
	position.y() = radius * sin(theta) * sin(phi);
	position.z() = radius * cos(theta);
}

Vector3d MagTracker::getPosition() {
	std::lock_guard<std::mutex> lock(mutex);
	return position;
}