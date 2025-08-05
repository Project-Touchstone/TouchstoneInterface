/**
 * DRIFTPlex.cpp - A group of DRIFT motors controlling a single node
 * Created by Carson G. Ray
 *
 * This file implements the DRIFTPlex class, which manages a set of DRIFTMotor
 * actuators to control a single haptic node. It provides methods for localization,
 * force control, collision handling, and kinematic updates.
 */

#include "DRIFTPlex.h"

using namespace std;
using namespace Utils;

/**
 * Attach motors and reference points to this DRIFTPlex.
 * @param motors      Pointer to array of DRIFTMotor objects.
 * @param homePoints  Pointer to array of home positions for each motor.
 * @param offsets     Pointer to array of offset vectors for each motor.
 */
void DRIFTPlex::attach(DRIFTMotor* motors, Vector3d* homePoints, Vector3d* offsets) {
    this->motors = motors;
    this->homePoints = homePoints;
    this->offsets = offsets;

    position = Vector3d::Zero();
    velocity = Vector3d::Zero();
}

/**
 * Update the orientation of the node (thread-safe).
 * @param orientation New orientation as a quaternion.
 */
void DRIFTPlex::updateOrientation(Quaterniond orientation) {
    std::lock_guard<std::mutex> lock(dataMutex);
    this->orientation = orientation;
}

/**
 * Update the position offset (thread-safe).
 * @param posOffset New position offset.
 */
void DRIFTPlex::updatePosOffset(Vector3d posOffset) {
    std::lock_guard<std::mutex> lock(dataMutex);
    this->posOffset = posOffset;
}

/**
 * Update the velocity offset (thread-safe).
 * @param velOffset New velocity offset.
 */
void DRIFTPlex::updateVelOffset(Vector3d velOffset) {
    std::lock_guard<std::mutex> lock(dataMutex);
    this->velOffset = velOffset;
}

/**
 * Get the home point for a given motor, including its offset.
 * @param motor Index of the motor.
 * @return Home point in global coordinates.
 */
Vector3d DRIFTPlex::getHomePoint(uint8_t motor) {
    return homePoints[motor] + getOffset(motor);
}

/**
 * Get the offset vector for a given motor, rotated by the current orientation.
 * @param motor Index of the motor.
 * @return Rotated offset vector.
 */
Vector3d DRIFTPlex::getOffset(uint8_t motor) {
    std::lock_guard<std::mutex> lock(dataMutex);
    return qRotate(orientation, offsets[motor]);
}

/**
 * Trilaterate the node position using three motors.
 * @param indices Array of three motor indices.
 * @param side    Side of the solution (+1 or -1).
 * @return solutionType containing position and score.
 */
DRIFTPlex::solutionType DRIFTPlex::trilaterate(uint8_t* indices, int8_t side) {
    Vector3d v1, v2, Xn, Yn, Zn, s;
    double r1, r2, r3, i, d, j, x, y, z, radicand;

    // Get string lengths from motors
    r1 = motors[indices[0]].getPosition();
    r2 = motors[indices[1]].getPosition();
    r3 = motors[indices[2]].getPosition();

    // Get baseline vectors between home points
    v1 = getHomePoint(indices[1]) - getHomePoint(indices[0]);
    v2 = getHomePoint(indices[2]) - getHomePoint(indices[0]);

    // Create local coordinate system for trilateration
    Xn = v1.normalized();
    Zn = v1.cross(v2).normalized();
    Yn = Xn.cross(Zn);

    i = Xn.dot(v2);
    d = Xn.dot(v1);
    j = Yn.dot(v2);

    // Solve for x, y, z using trilateration equations
    x = (pow(r1, 2) - pow(r2, 2) + pow(d, 2)) / (2 * d);
    y = (pow(r1, 2) - pow(r3, 2) + pow(i, 2) + pow(j, 2)) / (2 * j) - i / j * x;
    radicand = pow(r1, 2) - pow(x, 2) - pow(y, 2);
    z = sqrt(max(0., radicand)) * side;

    // Convert back to global coordinates
    Vector3d relPos3D = x * Xn + y * Yn + z * Zn;

    solutionType solution;
    solution.position = getHomePoint(indices[0]) + relPos3D;
    solution.score = exp(cbrt(radicand)); // Score based on solution quality
    return solution;
}

/**
 * Localize the node position using all motor combinations.
 * Uses weighted average of trilateration solutions.
 * @param stepTime Time step for velocity calculation (s).
 */
void DRIFTPlex::localize(double stepTime) {
    // Sum of position estimates
    Vector3d positionSum = Vector3d::Zero();
    double weightSum = 0;

    uint8_t combination[3] = {0, 1, 2};
    do
    {
        // For each combination of 3 motors, compute trilateration
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

/**
 * Set the force target to zero (convenience overload).
 */
void DRIFTPlex::setForceTarget() {
    Vector3d force = Vector3d::Zero();
    setForceTarget(force);
}

/**
 * Set the force target for the node (thread-safe).
 * @param force Desired force vector.
 */
void DRIFTPlex::setForceTarget(Vector3d force) {
    std::lock_guard<std::mutex> lock(dataMutex);
    this->forceTarget = force;
}

/**
 * Disable collision-based position limiting.
 */
void DRIFTPlex::disableCollisionControl() {
    collisionEnabled = false;
}

/**
 * Set the collision target for the node (thread-safe).
 * Enables collision control and sets the collision point, normal, and time to collision.
 * @param collisionPoint   Point of collision.
 * @param collisionNormal  Normal vector at collision.
 * @param timeToCollision  Time until collision (s).
 */
void DRIFTPlex::setCollisionTarget(Vector3d collisionPoint, Vector3d collisionNormal, double timeToCollision) {
    std::lock_guard<std::mutex> lock(dataMutex);
    collisionEnabled = true;
    this->collisionPoint = collisionPoint;
    this->collisionNormal = collisionNormal;
    this->timeToCollision = timeToCollision;
}

/**
 * Update the controller for all motors.
 * Sets force targets and applies collision limits if enabled.
 */
void DRIFTPlex::updateController() {
    std::vector<uint8_t> zeroForceMotors;

    Vector3d forceTargetCopy;
    {
        std::lock_guard<std::mutex> lock(dataMutex);
        forceTargetCopy = forceTarget;
    }
    if (forceTargetCopy.norm() == 0) {
        // If no force is requested, set all motors to zero force
        for (int i = 0; i < NUM_MOTORS; i++) {
            motors[i].setForceTarget(0);
            zeroForceMotors.push_back(i);
        }
    }
    else {
        // Compute string directions for each motor
        Matrix<double, 3, NUM_MOTORS> directions;
        for (int i = 0; i < NUM_MOTORS; i++) {
            directions.col(i) = (getPredictedPos() - getHomePoint(i)).normalized();
        }

        // Solve for force components for each string
        Vector<double, NUM_MOTORS> components = solveConstrainedForce(forceTargetCopy, directions);
        for (int i = 0; i < NUM_MOTORS; i++) {
            motors[i].setForceTarget(components(i));
            if (components(i) == 0) {
                zeroForceMotors.push_back(i);
            }
        }
    }
    // If collision control is enabled, apply position limits to zero-force motors
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
            if (vectorAtContact.dot(collisionNormalCopy) < 0) {
                // Applies relative position limit based on time to contact and reaction speed
                double posLimit = motors[i].getPosition() + timeToCollisionCopy * DRIFTMotor::getReactionSpeed();
                motors[i].setPositionLimit(posLimit);
            }
        }
    }
}

/**
 * Solve for string force components given a target force and string directions.
 * Uses SVD for particular solution and null space for constrained solution.
 * @param forceTarget Target force vector.
 * @param directions  Matrix of string direction vectors.
 * @return Vector of force components for each string.
 */
Vector<double, NUM_MOTORS> DRIFTPlex::solveConstrainedForce(Vector3d forceTarget, Matrix<double, 3, NUM_MOTORS> directions) {
    // Finds particular solution using SVD
    JacobiSVD<MatrixXd> svd(directions, ComputeThinU | ComputeThinV);

    Vector<double, NUM_MOTORS> particular = svd.solve(forceTarget);

    // Finds null space of the directions matrix
    FullPivLU<MatrixXd> lu(directions);
    MatrixXd nullSpace = lu.kernel();
    Vector<double, NUM_MOTORS> nullBasis = nullSpace.col(0);

    // Computes intersections with all zero planes to find valid solution
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
                    // Copies into minSolution
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

/**
 * Get the current node position (thread-safe), including position offset.
 * @return Node position vector.
 */
Vector3d DRIFTPlex::getPosition() {
    std::lock_guard<std::mutex> lock(dataMutex);
    return position + posOffset;
}

/**
 * Get the current node velocity (thread-safe), including velocity offset.
 * @return Node velocity vector.
 */
Vector3d DRIFTPlex::getVelocity() {
    std::lock_guard<std::mutex> lock(dataMutex);
    return velocity + velOffset;
}

/**
 * Get the predicted node position after a short time horizon.
 * @return Predicted position vector.
 */
Vector3d DRIFTPlex::getPredictedPos() {
    return getPosition() + getVelocity() * DRIFTMotor::getHorizonTime() / 1000000;
}

/**
 * Get the string length for a given motor, adjusted for node position.
 * @param motor Index of the motor.
 * @return Adjusted string length.
 */
double DRIFTPlex::getPosition(uint8_t motor) {
    Vector3d posCopy;
    {
        std::lock_guard<std::mutex> lock(dataMutex);
        posCopy = position;
    }
    double change = (getHomePoint(motor) - getPosition()).norm() - (getHomePoint(motor) - posCopy).norm();
    return motors[motor].getPosition() + change;
}

/**
 * Get the predicted string length for a given motor, using predicted node position.
 * @param motor Index of the motor.
 * @return Predicted string length.
 */
double DRIFTPlex::getPredictedPos(uint8_t motor) {
    Vector3d posCopy;
    {
        std::lock_guard<std::mutex> lock(dataMutex);
        posCopy = position;
    }
    double change = (getHomePoint(motor) - getPredictedPos()).norm() - (getHomePoint(motor) - posCopy).norm();
    return motors[motor].getPosition() + change;
}