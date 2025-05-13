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

void DRIFTPlex::updatePosOffset(Vector3d posOffset) {
	this->posOffset = posOffset;
}

void DRIFTPlex::updateVelOffset(Vector3d velOffset) {
	this->velOffset = velOffset;
}

Vector3d DRIFTPlex::getHomePoint(uint8_t motor) {
    return homePoints[motor] + getOffset(motor);
}

Vector3d DRIFTPlex::getOffset(uint8_t motor) {
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

	// Updates velocity
	velocity = (newPosition - position) / stepTime;
	position = newPosition;
}

void DRIFTPlex::setForceTarget() {
	Vector3d force = Vector3d::Zero();
    setForceTarget(force);
}

void DRIFTPlex::setForceTarget(Vector3d force) {
    setMode(FORCE);
    planeEnabled = false;
    this->forceTarget = force;
}

void DRIFTPlex::setPositionLimit(Vector3d posLimit, bool collision) {
    setMode(POSITION);
    planeEnabled = false;
    this->posLimit = posLimit;
    this->collision = collision;
}

void DRIFTPlex::setPlaneTarget(Vector3d planePoint, Vector3d planeNormal) {
    planeEnabled = true;
    this->planePoint = planePoint;
    this->planeNormal = planeNormal;
}

void DRIFTPlex::updateController(bool printing) {
    if (planeEnabled) {
        double distToPlane = (getPosition() - planePoint).dot(planeNormal);

        Vector3d n = distToPlane * planeNormal;
        if (distToPlane <= 0) {
            //If inside wall
            //Sets force target normal to wall
            setForceTarget(planeNormal * abs(distToPlane));
        }
        else {
            //If outside wall
            //Stops at closest point on wall
            Vector3d vhat = getVelocity().normalized();
            Vector3d slant = -distToPlane / vhat.dot(planeNormal) * vhat;
            setPositionLimit(getPosition() + slant, false);
        }
        planeEnabled = true;
    }
    Mode currentMode = getMode(); // Store the mode in a local variable to avoid re-evaluating it in the switch statement.
    switch (currentMode) {
        case FORCE: {
            Matrix<double, 3, NUM_MOTORS> directions;
            for (int i = 0; i < NUM_MOTORS; i++) {
                directions.col(i) = (getPredictedPos() - getHomePoint(i)).normalized();
            }

            Vector<double, NUM_MOTORS> components = solveConstrainedForce(forceTarget, directions, printing);
            for (int i = 0; i < NUM_MOTORS; i++) {
                motors[i].setForceTarget(components(i));
            }
            break;
        }
        case POSITION: {
            for (int i = 0; i < NUM_MOTORS; i++) {
                double motorPos = motors[i].getPosition();
                Vector3d currVector = getPosition() - getHomePoint(i);
                Vector3d currDiff = posLimit - getPosition();
                double currPos = currVector.norm();
                double newPos = (posLimit - getHomePoint(i)).norm();

                if (currVector.dot(currDiff) > 0 && (collision != (newPos > currPos))) {
                    motors[i].setPositionLimit(newPos - currPos + motorPos);
                } else {
                    motors[i].setForceTarget(0);
                }
            }
            break;
        }
    }
}

Vector<double, NUM_MOTORS> DRIFTPlex::solveConstrainedForce(Vector3d forceTarget, Matrix<double, 3, NUM_MOTORS> directions, bool printing) {
    //Finds particular solution
    JacobiSVD<MatrixXd> svd(directions, ComputeThinU | ComputeThinV);

    Vector<double, NUM_MOTORS> particular = svd.solve(forceTarget);

	//Finds null space
    FullPivLU<MatrixXd> lu(directions);
    MatrixXd nullSpace = lu.kernel();
    Vector<double, NUM_MOTORS> nullBasis = nullSpace.col(0);
    printf("Force Target: %.2f %.2f %.2f\n", forceTarget(0), forceTarget(1), forceTarget(2));
	printf("Particular: %.2f %.2f %.2f %.2f\n", particular(0), particular(1), particular(2), particular(3));
	printf("Null basis: %.2f %.2f %.2f %.2f\n", nullBasis(0), nullBasis(1), nullBasis(2), nullBasis(3));

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
				printf("Candidate Solution: %.2f %.2f %.2f %.2f\n", solution(0), solution(1), solution(2), solution(3));
				double sum = -solution.sum();
				if ((minSum == 0) || (sum < minSum)) {
                    minSum = sum;
                    //Copies into minSolution
                    for (int j = 0; j < NUM_MOTORS; j++) {
                        minSolution(j) = solution(j);
                    }
				}
			}
		}
    }

    Vector<double, NUM_MOTORS> components;
    if (solutionFound) {
        components = minSolution;
		printf("Solution: %.2f %.2f %.2f %.2f\n", components(0), components(1), components(2), components(3));
    }
    else {
        components = particular;
    }
    std::cout << endl;
    return components;
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
    return velocity + velOffset;
}

Vector3d DRIFTPlex::getPredictedPos() {
    return getPosition() + getVelocity()*DRIFTMotor::getHorizonTime()/1000000;
}

double DRIFTPlex::getPosition(uint8_t motor) {
	double change = (getHomePoint(motor) - getPosition()).norm() - (getHomePoint(motor) - position).norm();
    return motors[motor].getPosition() + change;
}

double DRIFTPlex::getPredictedPos(uint8_t motor) {
    double change = (getHomePoint(motor) - getPredictedPos()).norm() - (getHomePoint(motor) - position).norm();
	return motors[motor].getPosition() + change;
}