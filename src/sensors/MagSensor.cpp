#include "MagSensor.h"

MagSensor::MagSensor() : rawData{ 0, 0 } {}

void MagSensor::storeRawData(const std::array<int16_t, 2>& data) {
    std::lock_guard<std::mutex> lock(mutex);
    rawData = data;
}