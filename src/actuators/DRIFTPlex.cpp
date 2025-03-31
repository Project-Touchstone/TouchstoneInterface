/**
 * DRIFTPlex.cpp - A group of DRIFT motors controlling a single node
 * Created by Carson G. Ray
*/

#include "DRIFTPlex.h"

using namespace std;
using namespace Utils;

void DRIFTPlex::attach(DRIFTMotor* motors, Vector3d* homePoints, Vector3d* offsets, uint8_t numMotors) {
    this->motors = motors;
    this->homePoints = homePoints;
    this->numMotors = numMotors;
    this->offsets = offsets;

    position << 0, 0, 0;
    velocity << 0, 0, 0;
}

void DRIFTPlex::updateOffsets(Vector3d* offsets) {
    this->offsets = offsets;
}

Vector3d DRIFTPlex::getHomePoint(uint8_t motor) {
    return homePoints[motor] + offsets[motor];
}

Vector3d DRIFTPlex::trilaterate(vector<int> indices, uint8_t side) {
    Vector3d v1, v2, Xn, Yn, Zn, s;
    double r1, r2, r3, i, d, j, x, y, z;

    //Gets reference points
    r1 = motors[indices[0]].getPosition();
    r2 = motors[indices[1]].getPosition();
    r3 = motors[indices[2]].getPosition();

    //Gets baseline vectors  
    v1 = getHomePoint(1) - getHomePoint(0);
    v2 = getHomePoint(2) - getHomePoint(0);

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
    return getHomePoint(0) + relPos3D;
}

void DRIFTPlex::localize() {
    // Finds all possible trilaterations
    vector<Vector3d> solutions;
    // Minimum slack score
    float minSlack = 0;
    uint8_t minIdx = 0;

    vector<int> v;
    for (int i = 0; i < numMotors; i++) {
        v.push_back(i);
    }
    vector<int> combination = { 1, 2, 3 };

    uint8_t idx = 0;
    while (nextCombination(combination.begin(), combination.begin() + 3, v.end())) {
        Vector3d v1, v2;
        v1 = getHomePoint(combination[1]) - getHomePoint(combination[0]);
        v2 = getHomePoint(combination[2]) - getHomePoint(combination[0]);
        double val = -v1.cross(v2).dot(getHomePoint(combination[0]));
        uint8_t side = (int)(val / abs(val));
        solutions.push_back(trilaterate(combination, side));

        double slack = 0;
        for (int i = 0; i < numMotors; i++) {
            slack += pow((position - getHomePoint(i)).norm(), 2);
        }
        if (minSlack == 0 || slack < minSlack) {
            minSlack = slack;
            minIdx = idx;
        }
        idx++;
    }

    position = solutions[minIdx];

    for (int i = 0; i < numMotors; i++) {
        slants(i, all) = (position - getHomePoint(i)).normalized();
    }
    
    Vector3d slantVel;
    for (int i = 0; i < numMotors; i++) {
        motors[i].sampleVelocity();
        slantVel(i) = motors[i].getVelocity();
    }
    
    JacobiSVD<MatrixXd> svd(slants, ComputeThinU | ComputeThinV);
    
    velocity = svd.solve(slantVel);
}

void DRIFTPlex::setForceTarget() {
    Vector3d force;
    force << 0, 0, 0;
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
            for (int i = 0; i < numMotors; i++) {
                motors[i].setForceTarget(forceTarget.dot(slants(i, all)));
            }
            break;
        case POSITION:
            for (int i = 0; i < numMotors; i++) {
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
    return position;
}

Vector3d DRIFTPlex::getVelocity() {
    return velocity;
}

Vector3d DRIFTPlex::getPredictedPos() {
    return getPosition() + getVelocity()*DRIFTMotor::getHorizonTime()/1000000;
}

double DRIFTPlex::getPredictedPos(uint8_t motor) {
    return motors[motor].getPosition() + getVelocity().dot(slants(motor, all))*DRIFTMotor::getHorizonTime()/1000000;
}