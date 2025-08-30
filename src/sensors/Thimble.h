#ifndef THIMBLE_H
#define THIMBLE_H

#include "MagTracker.h"
#include "IMU.h"
#include "../utils/Utils.h"

class Thimble {
public:
    // Attach magnetic trackers
    void attach(MagTracker* trackers, IMU* imu);

    // Update thimble state
    void update(double stepTime);

    // Gets airgap between inner and outer caps
    double getAirGap();

	// Get inner cap position
    Vector3d getInnerCapPos();

    // Gets inner cap velocity
	Vector3d getInnerCapVel();

	// Get inner cap angular velocity
	Vector3d getInnerCapAngVel();

	// Get inner cap orientation
	Quaterniond getInnerCapOrient();

    // Get outer cap orientation
    Quaterniond getOuterCapOrient();

private:
    // Magnetic tracker objects
    MagTracker* magTrackers = nullptr; // Not owned, do not delete

    // IMU objects
    IMU* imu = nullptr;

	// Inner cap radius
	const double innerCapRadius = 9.625;

    // Outer cap radius
	const double outerCapRadius = 15.175;

    // Air gap between inner and outer caps
    const double airGap = 2.625;

    // Inner cap orientation
	Quaterniond innerCapOrient = Quaterniond::Identity();

    // Inner cap angular velocity
	Vector3d innerCapAngVel = Vector3d::Zero();

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