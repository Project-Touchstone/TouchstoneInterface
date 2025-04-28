#ifndef MAGSENSOR_H
#define MAGSENSOR_H

#include <mutex>
#include <array>

class MagSensor {
public:
    MagSensor();
    virtual ~MagSensor() = default;

    // Store raw sensor data
    void storeRawData(const std::array<int16_t, 2>& data);

    // Update sensor data (to be implemented by derived classes)
    virtual void updateData() = 0;

protected:
    std::array<int16_t, 2> rawData;
    std::mutex mutex;
};

#endif // MAGSENSOR_H