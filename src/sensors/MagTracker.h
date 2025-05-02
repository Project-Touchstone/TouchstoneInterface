#ifndef MAGTRACKER_H
#define MAGTRACKER_H

//External imports
#include <mutex>
#include <array>
#include <Eigen/Dense>

//Local imports
#include "../utils/Utils.h"

using namespace Eigen;

class MagTracker {
    public:
        // Sets sensor orientation offset
        void setSensorOrientation(Quaterniond sensorOrient);
        // Store raw sensor data
        void storeRawData(const std::array<int16_t, 3>& data);
        
        // Update sensor data using relative magnet orientation
        void updateData(Quaterniond magOrient);
        // Get relative position vector
        Vector3d getPosition();

    private:
        // Raw sensor data
        Vector3d sensorData = Vector3d::Zero();
        // Sensor data mutex
        std::mutex mutex;
        //Sensor data multiplier
        const double magSensorMultiplier = 0.098;
        // Sensor orientation offset
        Quaterniond sensorOrient = Quaterniond::Identity();
        //Relative position from magnet
        Vector3d position = Vector3d::Zero();
};

#endif // MAGTRACKER_H