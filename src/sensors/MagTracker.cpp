#include "MagTracker.h"

using namespace Eigen;
using namespace Utils;

void MagTracker::setSensorOrientation(Quaterniond sensorOrient) {
	this->sensorOrient = sensorOrient;
}

void MagTracker::setInitialPosition(Vector3d position) {
	this->position = position;
}

void MagTracker::storeRawData(const std::array<int16_t, 3>& data) {
    std::lock_guard<std::mutex> lock(dataMutex);
	for (uint8_t i = 0; i < 3; i++) {
		sensorData(i) = static_cast<double>(data[i]) * magSensorMultiplier * axisDirs(i);
	}
}

void MagTracker::updateData(Quaterniond magOrient, bool printing) {
	//Transforms sensor data from sensor reference frame to magnet reference frame
	Vector3d Bfield = qRotate(sensorOrient * magOrient.conjugate(), sensorData);

	if (Bfield.norm() < 1.0) {
		return;
	}
	//Gets lateral and vertical components of magnetic field
	double Bc = sqrt(pow(Bfield.x(),2) + pow(Bfield.y(),2));
	double Bz = Bfield.z();

	//Gets old theta angle and associated sign change
	double prevTheta = acos(position.normalized().dot(Vector3d(0, 0, 1)));
	//Gets sign of cosine of angle
	int sign = -1;
	if (cos(prevTheta) > 0) {
		sign = 1;
	}

	//Calculates theta angle
	double theta = 0;
	//Checks for special cases
	if (Bc == 0) {
		theta = 0;
	} else if (Bz == 0) {
		theta = acos(sign * sqrt(1. / 3.));
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
		double root = (-b + axisSide * sqrt(pow(b, 2) - 4 * a * c)) / (2 * a);
		theta = acos(sign * sqrt(root));
	}

	//Calculates phi angle
	double phi = atan2(Bfield.y(), Bfield.x());
	if (sign < 0) {
		phi = atan2(-Bfield.y(), -Bfield.x());
	}

	//Calculates radius
	double radius = 0;
	if (Bc != 0) {
		radius = abs(cbrt(1 / Bc * 3 * cos(theta) * sin(theta)));
	}
	else if (Bz != 0) {
		radius = abs(cbrt(1 / Bz * (3 * pow(cos(theta), 2) - 1)));
	}
	else {
		radius = 0;
	}
	//Combines spherical coordinates to get final relative position vector
	std::lock_guard<std::mutex> lock(dataMutex);
	position.x() = radius * sin(theta) * cos(phi);
	position.y() = radius * sin(theta) * sin(phi);
	position.z() = radius * cos(theta);
	if (printing) {
		//printf("Magnetic Field: (%.2f, %.2f, %.2f)\n", Bfield.x(), Bfield.y(), Bfield.z());
		//printf("Position: (%.2f, %.2f, %.2f)\n", position.x(), position.y(), position.z());
		//printf("Theta: %.2f\n", theta * 180 / EIGEN_PI);
		//printf("Phi: %.2f\n", phi * 180 / EIGEN_PI);
	}
}

Vector3d MagTracker::getPosition() {
	std::lock_guard<std::mutex> lock(dataMutex);
	return position;
}