/**
 * DRIFTPlex.cpp - A group of DRIFT motors controlling a single node
 * Created by Carson G. Ray
*/

#include "DRIFTPlex.h"

using namespace std;
using namespace Utils;

void DRIFTPlex::attach(DRIFTMotor* motors, Vector3d* homePoints, Vector3d* offsets) {
    this->motors = motors;
    this->homePoints = homePoints;
    this->offsets = offsets;

    position = Vector3d::Zero();
    velocity = Vector3d::Zero();
}

void DRIFTPlex::updateOrientation(Quaterniond orientation) {
    std::lock_guard<std::mutex> lock(dataMutex);
    this->orientation = orientation;
}

void DRIFTPlex::updatePosOffset(Vector3d posOffset) {
    std::lock_guard<std::mutex> lock(dataMutex);
	this->posOffset = posOffset;
}

void DRIFTPlex::updateVelOffset(Vector3d velOffset) {
    std::lock_guard<std::mutex> lock(dataMutex);
	this->velOffset = velOffset;
}

Vector3d DRIFTPlex::getHomePoint(uint8_t motor) {
    return homePoints[motor] + getOffset(motor);
}

Vector3d DRIFTPlex::getOffset(uint8_t motor) {
    std::lock_guard<std::mutex> lock(dataMutex);
	return qRotate(orientation, offsets[motor]);
}

DRIFTPlex::solutionType DRIFTPlex::trilaterate(uint8_t* indices, int8_t side) {
    Vector3d v1, v2, Xn, Yn, Zn, s;
    double r1, r2, r3, i, d, j, x, y, z, radicand;

    //Gets reference points
    r1 = motors[indices[0]].getPosition();
    r2 = motors[indices[1]].getPosition();
    r3 = motors[indices[2]].getPosition();

    //Gets baseline vectors  
    v1 = getHomePoint(indices[1]) - getHomePoint(indices[0]);
    v2 = getHomePoint(indices[2]) - getHomePoint(indices[0]);

    // Creates coordinate system relative to shared plane
    Xn = v1.normalized();
    Zn = v1.cross(v2).normalized();
    Yn = Xn.cross(Zn);

    i = Xn.dot(v2);
    d = Xn.dot(v1);
    j = Yn.dot(v2);

    x = (pow(r1, 2) - pow(r2, 2) + pow(d, 2)) / (2 * d);
    y = (pow(r1, 2) - pow(r3, 2) + pow(i, 2) + pow(j, 2)) / (2 * j) - i / j * x;
    radicand = pow(r1, 2) - pow(x, 2) - pow(y, 2);
    z = sqrt(max(0., radicand))*side;

    // Converts back to global coordinate system
    Vector3d relPos3D = x * Xn + y * Yn + z * Zn;

    solutionType solution;
    solution.position = getHomePoint(indices[0]) + relPos3D;
    solution.score = exp(cbrt(radicand));
    return solution;
}

void DRIFTPlex::localize(double stepTime) {
    // Sum of position estimates
    Vector3d positionSum = Vector3d::Zero();
    
    double weightSum = 0;

    uint8_t combination[3] = {0, 1, 2};
    do
    {
        Vector3d v1, v2;
        v1 = getHomePoint(combination[1]) - getHomePoint(combination[0]);
        v2 = getHomePoint(combination[2]) - getHomePoint(combination[0]);
        double val = -v1.cross(v2).dot(getHomePoint(combination[0]));
        int8_t side = (int8_t)(val / abs(val));
        solutionType solution = trilaterate(combination, side);
        double weight = solution.score;
        positionSum += solution.position * weight;
        weightSum += weight;
    } while (nextCombination(NUM_MOTORS, 3, combination));

    Vector3d newPosition = positionSum / weightSum;

    // Only lock for assignment
    {
        std::lock_guard<std::mutex> lock(dataMutex);
        velocity = (newPosition - position) / stepTime;
        position = newPosition;
    }
}

void DRIFTPlex::setForceTarget() {
	Vector3d force = Vector3d::Zero();
    setForceTarget(force);
}

void DRIFTPlex::setForceTarget(Vector3d force) {
    std::lock_guard<std::mutex> lock(dataMutex);
    this->forceTarget = force;
}

void DRIFTPlex::disableCollisionControl() {
    collisionEnabled = false;
}

void DRIFTPlex::setCollisionTarget(Vector3d collisionPoint, Vector3d collisionNormal, double timeToCollision) {
    std::lock_guard<std::mutex> lock(dataMutex);
    collisionEnabled = true;
    this->collisionPoint = collisionPoint;
    this->collisionNormal = collisionNormal;
	this->timeToCollision = timeToCollision;
}

void DRIFTPlex::updateController() {
    std::vector<uint8_t> zeroForceMotors;

    Vector3d forceTargetCopy;
    {
        std::lock_guard<std::mutex> lock(dataMutex);
        forceTargetCopy = forceTarget;
    }
	if (forceTargetCopy.norm() == 0) {
		for (int i = 0; i < NUM_MOTORS; i++) {
			motors[i].setForceTarget(0);
            zeroForceMotors.push_back(i);
		}
    }
    else {
        Matrix<double, 3, NUM_MOTORS> directions;
        for (int i = 0; i < NUM_MOTORS; i++) {
            directions.col(i) = (getPredictedPos() - getHomePoint(i)).normalized();
        }

        Vector<double, NUM_MOTORS> components = solveConstrainedForce(forceTargetCopy, directions);
        for (int i = 0; i < NUM_MOTORS; i++) {
            motors[i].setForceTarget(components(i));
            if (components(i) == 0) {
                zeroForceMotors.push_back(i);
            }
        }
    }
    if (collisionEnabled && zeroForceMotors.size() > 0) {
        Vector3d collisionPointCopy, collisionNormalCopy;
        float timeToCollisionCopy;
        {
            std::lock_guard<std::mutex> lock(dataMutex);
            collisionPointCopy = collisionPoint;
            collisionNormalCopy = collisionNormal;
            timeToCollisionCopy = timeToCollision;
        }
    
        for (uint8_t i: zeroForceMotors) {
            // Gets string vector at contact point
            Vector3d vectorAtContact = collisionPointCopy - getHomePoint(i);
            
            // Determines whether vector is relevant to collision normal
            if (-vectorAtContact.dot(collisionNormalCopy) > 0) {
                // Applies relative position limit based on time to contact and reaction speed
                double posLimit = motors[i].getPosition() + timeToCollisionCopy * DRIFTMotor::getReactionSpeed();
                motors[i].setPositionLimit(posLimit);
            }
        }
    }
}

Vector<double, NUM_MOTORS> DRIFTPlex::solveConstrainedForce(Vector3d forceTarget, Matrix<double, 3, NUM_MOTORS> directions) {
    //Finds particular solution
    JacobiSVD<MatrixXd> svd(directions, ComputeThinU | ComputeThinV);

    Vector<double, NUM_MOTORS> particular = svd.solve(forceTarget);

	//Finds null space
    FullPivLU<MatrixXd> lu(directions);
    MatrixXd nullSpace = lu.kernel();
    Vector<double, NUM_MOTORS> nullBasis = nullSpace.col(0);

    //Computes intersections with all zero planes
    double minSum = 0;
	Vector<double, NUM_MOTORS> minSolution = Vector<double, NUM_MOTORS>::Zero();
    bool solutionFound = false;
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
		if (nullBasis(i) != 0) {
			double intersection = -particular(i) / nullBasis(i);
			Vector<double, NUM_MOTORS> solution = particular + intersection * nullBasis;
            bool valid = true;
			for (int j = 0; j < NUM_MOTORS; j++) {
				valid &= (solution(j) <= 0);
			}
			if (valid) {
                solutionFound = true;
				double sum = -solution.sum();
				if ((minSum == 0) || (sum < minSum)) {
                    minSum = sum;
                    //Copies into minSolution
                    minSolution = solution;
				}
			}
		}
    }

    Vector<double, NUM_MOTORS> components;
    if (solutionFound) {
        components = minSolution;
    }
    else {
        components = particular;
    }
    return components;
}

Vector3d DRIFTPlex::getPosition() {
	std::lock_guard<std::mutex> lock(dataMutex);
    return position + posOffset;
}

Vector3d DRIFTPlex::getVelocity() {
	std::lock_guard<std::mutex> lock(dataMutex);
    return velocity + velOffset;
}

Vector3d DRIFTPlex::getPredictedPos() {
    return getPosition() + getVelocity()*DRIFTMotor::getHorizonTime()/1000000;
}

double DRIFTPlex::getPosition(uint8_t motor) {
    Vector3d posCopy;
    {
        std::lock_guard<std::mutex> lock(dataMutex);
        posCopy = position;
    }
	double change = (getHomePoint(motor) - getPosition()).norm() - (getHomePoint(motor) - posCopy).norm();
    return motors[motor].getPosition() + change;
}

double DRIFTPlex::getPredictedPos(uint8_t motor) {
    Vector3d posCopy;
    {
        std::lock_guard<std::mutex> lock(dataMutex);
        posCopy = position;
    }
    double change = (getHomePoint(motor) - getPredictedPos()).norm() - (getHomePoint(motor) - posCopy).norm();
	return motors[motor].getPosition() + change;
}