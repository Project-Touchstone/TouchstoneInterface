#include "MagTracker.h"
#include <iostream>

MagTracker::MagTracker() {}

void MagTracker::updateData() {
    std::lock_guard<std::mutex> lock(mutex);
    // Process rawData and update tracker-specific state
    std::cout << "Updating MagTracker data: " << rawData[0] << ", " << rawData[1] << std::endl;
}