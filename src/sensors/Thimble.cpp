#include "Thimble.h"
#include <iostream>

using namespace Utils;

void Thimble::attachMagTrackers(MagTracker* trackers) {
    magTrackers = trackers;
}

void Thimble::update() {
	// Updates magnetic trackers
	for (int i = 0; i < 2; ++i) {
		magTrackers[i].updateData(innerCapOrient);
	}

	// Gets radius vectors
	Vector3d r1 = magTrackers[0].getPosition();
	Vector3d r2 = magTrackers[1].getPosition();

	// Solves quadratic to get new scale factor
	double a = pow((r1 - r2).norm(), 2) / 4;
	double b = innerCapRadius * (r1.z() - r2.z());
	double c = pow(innerCapRadius, 2) - pow(outerCapRadius, 2);

	// Chooses solution based on previous scale factor
	int sign = -1;
	if (prevScaleFactor > -b / (2 * a)) {
		sign = 1;
	}

	double scaleFactor = (-b + sign * pow(pow(b, 2) - 4 * a * c, 0.5)) / (2 * a);

	// Updates previous scale factor
	prevScaleFactor = scaleFactor;

	// Updates radius vectors
	r1 *= scaleFactor;
	r2 *= scaleFactor;

	// Inner radius vector
	Vector3d innerVector = innerCapRadius * Vector3d(0, 0, 1);

	// Finds baseline vector
	Vector3d baseline = (r1 - r2) / 2 + innerVector;

	// Gets inner cap orientation relative to baseline refere ce
	innerCapOrient = Quaterniond::FromTwoVectors(innerVector, baseline);

	// Finds inner cap position relative to baseline reference
	innerCapPos = qRotate(innerCapOrient.conjugate(), -(r1 + r2) / 2);
}

Vector3d Thimble::getInnerCapPos() {
	return innerCapPos;
}

Quaterniond Thimble::getInnerCapOrient() {
	return innerCapOrient;
}