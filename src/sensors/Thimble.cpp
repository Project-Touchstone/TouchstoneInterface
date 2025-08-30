#include "Thimble.h"
#include "IMU.h"
#include <iostream>

using namespace Utils;

void Thimble::attach(MagTracker* trackers, IMU* imu) {
	magTrackers = trackers;
	this->imu = imu;
}

void Thimble::update(double stepTime) {
	//Updates imu orientation
	imu->updateOrientation(stepTime);

	// Rotational transform between magnets
	Quaterniond rotTransform = Quaterniond(0, 0, 1, 0);
	// Updates magnetic trackers
	for (int i = 0; i < 2; ++i) {
		Quaterniond magOrient = innerCapOrient;
		if (i == 1) {
			magOrient = innerCapOrient * rotTransform;
		}
		magTrackers[i].updateData(innerCapOrient);
	}

	// Gets radius vectors
	Vector3d r1 = magTrackers[0].getPosition();
	Vector3d r2 = magTrackers[1].getPosition();
	// Inverts z-axis on second magnet
	r2.z() *= -1;

	if (r1.norm() == 0 || r2.norm() == 0) {
		return;
	}

	// Solves quadratic to get new scale factor
	double a = pow((r1 - r2).norm(), 2) / 4;
	double b = innerCapRadius * (r1.z() - r2.z());
	double c = pow(innerCapRadius, 2) - pow(outerCapRadius, 2);

	// Chooses solution based on previous scale factor
	int sign = -1;
	if (prevScaleFactor > -b / (2 * a)) {
		sign = 1;
	}

	double scaleFactor = (-b + sign * sqrt(pow(b, 2) - 4 * a * c)) / (2 * a);

	// Updates previous scale factor
	prevScaleFactor = scaleFactor;

	// Updates radius vectors
	r1 *= scaleFactor;
	r2 *= scaleFactor;

	// Inner radius vector
	Vector3d innerVector = innerCapRadius * Vector3d(0, 0, 1);

	// Finds baseline vector
	Vector3d baseline = (r1 - r2) / 2 + innerVector;

	std::lock_guard<std::mutex> lock(dataMutex);
	// Gets inner cap orientation relative to baseline reference
	Quaterniond newInnerCapOrient = Quaterniond::FromTwoVectors(baseline, innerVector).normalized();

	// Updates inner cap angular velocity
	AngleAxisd deltaAngle = AngleAxisd(newInnerCapOrient * innerCapOrient.conjugate());
	innerCapAngVel = deltaAngle.axis() * deltaAngle.angle() / stepTime;

	// Updates inner cap orientation
	innerCapOrient = newInnerCapOrient;

	// Finds inner cap position relative to baseline reference
	Vector3d newInnerCapPos = qRotate(innerCapOrient, -(r1 + r2) / 2);
	// Updates velocity
	innerCapVel = (newInnerCapPos - innerCapPos) / stepTime;
	// Updates inner cap position
	innerCapPos = newInnerCapPos;
}

Vector3d Thimble::getInnerCapPos() {
    std::lock_guard<std::mutex> lock(dataMutex);
    return innerCapPos;
}

Quaterniond Thimble::getInnerCapOrient() {
    std::lock_guard<std::mutex> lock(dataMutex);
    return innerCapOrient;
}

Quaterniond Thimble::getOuterCapOrient() {
	return imu->getOrientation();
}

Vector3d Thimble::getInnerCapVel() {
    std::lock_guard<std::mutex> lock(dataMutex);
    return innerCapVel;
}

Vector3d Thimble::getInnerCapAngVel() {
	std::lock_guard<std::mutex> lock(dataMutex);
	return innerCapAngVel;
}