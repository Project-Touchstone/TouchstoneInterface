#include "Thimble.h"
#include <iostream>

void Thimble::attachMagTrackers(MagTracker* trackers) {
    magTrackers = trackers;
}

void Thimble::update() {
	// Update the state of the thimble using the attached magnetic trackers
	for (int i = 0; i < 2; ++i) {
		magTrackers[i].updateData();
	}

	// Additional logic to process the data from the trackers can be added here
	std::cout << "Thimble updated with magnetic tracker data." << std::endl;
}