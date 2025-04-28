#ifndef THIMBLE_H
#define THIMBLE_H

#include "MagTracker.h"

class Thimble {
public:
    // Attach magnetic trackers
    void attachMagTrackers(MagTracker* trackers);

    // Update thimble state
    void update();

private:
    MagTracker* magTrackers;
};

#endif // THIMBLE_H