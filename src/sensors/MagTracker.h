#ifndef MAGTRACKER_H
#define MAGTRACKER_H

#include "MagSensor.h"

class MagTracker : public MagSensor {
public:
    MagTracker();
    ~MagTracker() override = default;

    // Update sensor data
    void updateData() override;
};

#endif // MAGTRACKER_H