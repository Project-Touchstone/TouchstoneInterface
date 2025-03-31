/**
 * utils.cpp - Useful functions
 * Created by Carson G. Ray
*/

#include "Utils.h"

uint16_t Utils::factorial(uint16_t input) {
    if (input < 0) {
        return 0;
    }

    uint16_t result = 1;
    for (uint16_t next = 1; next <= input; next++) {
        result *= next;
    }
    return result;
}

uint16_t Utils::combinations(uint16_t n, uint16_t r) {
    if (r > n || r < 0) {
        return 0;
    }
    
    return factorial(n) / (factorial(r) * factorial(n - r));
}

string Utils::toString(const MatrixXd mat) {
    stringstream ss;
    ss << mat;
    return ss.str().c_str();
}

void Utils::sleep(uint32_t ms) {
    this_thread::sleep_for(std::chrono::milliseconds(ms));
}