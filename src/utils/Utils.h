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
	string toString(const MatrixXd mat);
	void sleep(uint32_t ms);
	bool nextCombination(uint8_t n, uint8_t r, uint8_t* indices);

	Quaterniond qScalarMult(Quaterniond q, double scalar);
	Quaterniond qAdd(Quaterniond q1, Quaterniond q2);
	Matrix3d skewSymmetric(Vector3d vector);
}

#endif