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
        // Sets initial relative position
        void setInitialPosition(Vector3d position);
        // Store raw sensor data
        void storeRawData(const std::array<int16_t, 3>& data);
        
        // Update sensor data using relative magnet orientation
        void updateData(Quaterniond magOrient, bool printing);
        // Get relative position vector
        Vector3d getPosition();

    private:
        // Raw sensor data
        Vector3d sensorData = Vector3d::Zero();
        // Sensor axis directions
		Vector3d axisDirs = Vector3d(1, 1, -1);
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