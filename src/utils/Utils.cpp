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

bool Utils::nextCombination(uint8_t n, uint8_t r, uint8_t* indices) {
    if (n <= r) {
        return false;
    }

    for (int8_t i = r - 1; i >= 0; i--) {
        int thres;
        if (i == r - 1) {
            thres = n;
        }
        else {
            thres = indices[i + 1];
        }
        if (indices[i] < thres - 1) {
            indices[i]++;
            return true;
        }
    }
    return false;
}

string Utils::toString(const MatrixXd mat) {
    stringstream ss;
    ss << mat;
    return ss.str().c_str();
}

void Utils::sleep(uint32_t ms) {
    this_thread::sleep_for(std::chrono::milliseconds(ms));
}