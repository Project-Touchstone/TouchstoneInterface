/**
 * Utils.h - Useful functions
 * Created by Carson G. Ray
*/

#ifndef Utils_h
#define Utils_h

//External imports
#include <math.h>
#include <Eigen/Dense>
#include <stdint.h>
#include <chrono>
#include <thread>

using namespace std;
using namespace Eigen;

namespace Utils {
	uint16_t factorial(uint16_t input);
	uint16_t combinations(uint16_t n, uint16_t r);
	string toString(MatrixXd mat);
	void sleep(uint32_t ms);

    template <typename Iterator>
    bool nextCombination(Iterator first, Iterator k, Iterator last);
}

template <typename Iterator>
bool Utils::nextCombination(Iterator first, Iterator k, Iterator last) {
    if (first == last || first == k || last == k) {
        return false;
    }
    if (first == last || first == k || last == k) {
        return false;
    }

    Iterator i = k;
    --i;

    while (true) {
        if (*i < *last - 1) {
            ++(*i);
            for (Iterator j = i; j < k; ) {
                ++j;
                *j = *i;
                ++(*j);
            }
            return true;
        }
        if (i == first) {
            return false;
        }
        --i;
        --last;
    }
}

#endif