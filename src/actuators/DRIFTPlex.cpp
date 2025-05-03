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
    this->orientation = orientation;
}

void DRIFTPlex::updatePositionOffset(Vector3d posOffset) {
	this->posOffset = posOffset;
}

Vector3d DRIFTPlex::getHomePoint(uint8_t motor) {
    return homePoints[motor] + getOffset(motor);
}

Vector3d DRIFTPlex::getOffset(uint8_t motor) {
	return qRotate(orientation, offsets[motor]);
}

DRIFTPlex::solutionType DRIFTPlex::trilaterate(uint8_t* indices, int8_t side) {
    Vector3d v1, v2, Xn, Yn, Zn, s;
    double r1, r2, r3, i, d, j, x, y, z;

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
    z = sqrt(max(0., pow(r1, 2)-pow(x, 2)-pow(y,2)))*side;

    // Converts back to global coordinate system
    Vector3d relPos3D = x * Xn + y * Yn + z * Zn;

    solutionType solution;
    solution.position = getHomePoint(indices[0]) + relPos3D;
    solution.z = abs(z);
    return solution;
}

void DRIFTPlex::localize() {
    // New position vector
    Vector3d newPosition = Vector3d::Zero();
    
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
        double weight = exp(abs(solution.z));
        newPosition += solution.position * weight;
        weightSum += weight;
    } while (nextCombination(NUM_MOTORS, 3, combination));

    position = newPosition / weightSum;

    for (int i = 0; i < NUM_MOTORS; i++) {
        slants(i, all) = (position - getHomePoint(i)).normalized();
    }
    
    Vector<double, NUM_MOTORS> slantVel;
    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].sampleVelocity();
        slantVel(i) = motors[i].getVelocity();
    }
    
    JacobiSVD<MatrixXd> svd(slants, ComputeThinU | ComputeThinV);
    
    velocity = svd.solve(slantVel);
}

void DRIFTPlex::setForceTarget() {
	Vector3d force = Vector3d::Zero();
    setForceTarget(force);
}

void DRIFTPlex::setForceTarget(Vector3d force) {
    setMode(FORCE);
    this->forceTarget = force;
}

void DRIFTPlex::setPositionLimit(Vector3d posLimit, bool collision) {
    setMode(POSITION);
    this->posLimit = posLimit;
    this->collision = collision;
}

void DRIFTPlex::updateController() {
    switch(getMode()) {
        case FORCE:
            for (int i = 0; i < NUM_MOTORS; i++) {
                motors[i].setForceTarget(forceTarget.dot(slants(i, all)));
            }
            break;
        case POSITION:
            for (int i = 0; i < NUM_MOTORS; i++) {
                double motorPos = motors[i].getPosition();
                double currPos = (position - getHomePoint(i)).norm();
                double newPos = (posLimit - getHomePoint(i)).norm();
                if (collision != (newPos > currPos)) {
                    motors[i].setPositionLimit(newPos - (currPos - motorPos));
                } else {
                    motors[i].setForceTarget(0);
                }
            }
            break;
    }
}

void DRIFTPlex::setMode(Mode mode) {
    this->mode = mode;
}

DRIFTPlex::Mode DRIFTPlex::getMode() {
    return mode;
}

Vector3d DRIFTPlex::getPosition() {
    return position + posOffset;
}

Vector3d DRIFTPlex::getVelocity() {
    return velocity;
}

Vector3d DRIFTPlex::getPredictedPos() {
    return getPosition() + getVelocity()*DRIFTMotor::getHorizonTime()/1000000;
}

double DRIFTPlex::getPredictedPos(uint8_t motor) {
    Vector3d change = posOffset + getVelocity() * DRIFTMotor::getHorizonTime() / 1000000;
    return motors[motor].getPosition() + change.dot(slants(motor, all));
}

double DRIFTPlex::estimateRotationChange(Quaterniond axis, double predictedDelta) {
    double weightSum = 0;
	double deltaAngleSum = 0;

	for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        //Gets angle solution using tangent approximation

        //Gets relative vector from center of thimble to home point
        Vector3d relativePos = qRotate(axis.conjugate(), getHomePoint(i) - position);

        //Gets actual and predicted string lengths
        double actualDist = motors[i].getPosition();
        double predictedDist = relativePos.norm();

        //Transforms home points and offsets to rotation axis reference
        Vector3d localOffset = qRotate(axis.conjugate(), getOffset(i));
        
        //Gets projection of offset vector into rotation plane
        Vector3d rVector = Vector3d(localOffset.x(), localOffset.y(), 0);

        //Gets component normal to rotation plane
        Vector3d nVector = Vector3d(0, 0, localOffset.z());

        //Gets radius of rotation circle
        double radius = rVector.norm();

        //Gets current rotation angle
        double angle = atan2(localOffset.y(), localOffset.x());

        //Solves quadratic to find change in angle
        double a = pow(radius, 2);
        double b = -2 * radius * (relativePos.x() * cos(angle) + relativePos.y() * sin(angle));
        double c = pow(predictedDist, 2) - pow(actualDist, 2);

        //Ensures solution is real
        double discriminant = pow(b, 2) - 4 * a * c;
        if (discriminant >= 0) {
            int sign = -1;
            //Determines which solution to use by comparing to predicted value
            if (predictedDelta > -b / (2 * a)) {
                sign = 1;
            }
            double deltaAngle = (-b + sign * pow(discriminant, 0.5)) / (2 * a);

            //Weights based on how much rotation effects motor distance
            double weight = exp(relativePos.cross(rVector).norm());
            deltaAngleSum += deltaAngle * weight;
            weightSum += weight;
        }
	}
    
    // If valid solution was found perform weighted average
    if (weightSum > 0) {
        return deltaAngleSum / weightSum;
    }
    else {
        // Otherwise just return prediction
        return predictedDelta;
    }
}