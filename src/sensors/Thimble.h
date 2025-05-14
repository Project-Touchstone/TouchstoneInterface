#ifndef THIMBLE_H
#define THIMBLE_H

#include "MagTracker.h"
#include "IMU.h"
#include "../utils/Utils.h"

class Thimble {
public:
    // Attach magnetic trackers
    void attachMagTrackers(MagTracker* trackers);

    // Update thimble state
    void update(double stepTime, bool printing);

	// Get inner cap position
    Vector3d getInnerCapPos();

    // Gets inner cap velocity
	Vector3d getInnerCapVel();

	// Get inner cap orientation
	Quaterniond getInnerCapOrient();

private:
    // Magnetic tracker objects
    MagTracker* magTrackers;

	// Inner cap radius
	double innerCapRadius = 9.625;

    // Outer cap radius
	double outerCapRadius = 15.175;

    // Inner cap orientation
	Quaterniond innerCapOrient = Quaterniond::Identity();

    // Inner cap relative position
	Vector3d innerCapPos = Vector3d::Zero();

    // Inner cap velocity
	Vector3d innerCapVel = Vector3d::Zero();

    //Previous scale factor
    double prevScaleFactor = 1;

    //Mutex
	std::mutex dataMutex;
};

#endif // THIMBLE_H