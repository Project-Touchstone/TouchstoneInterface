/**
 * HydraPlex.cpp - A group of DRIFT motors controlling a single node
 * Created by Carson G. Ray
 *
 * This file implements the HydraPlex class, which manages a set of HydraFOCMotor
 * actuators to control a single haptic node. It provides methods for localization,
 * force control, collision handling, and kinematic updates.
 */

#include "HydraPlex.h"

using namespace std;
using namespace Utils;

const double HydraPlex::horizonTime = 0.01;
const double HydraPlex::minForce = 0.1;
const double HydraPlex::controllerGain = 0.1;

/**
 * Attach motors and reference points to this HydraPlex.
 * @param motors      Pointer to array of HydraFOCMotor objects.
 * @param thimble     Pointer to thimble object
 * @param homePoints  Pointer to array of home positions for each motor.
 * @param offsets     Pointer to array of offset vectors for each motor.
 */
void HydraPlex::attach(HydraFOCMotor* motors, Thimble* thimble, Vector3d* homePoints, Vector3d* offsets) {
    this->motors = motors;
    this->thimble = thimble;
    this->homePoints = homePoints;
    this->offsets = offsets;

    position = Vector3d::Zero();
    velocity = Vector3d::Zero();
}

/**
 * Update the orientation of the node (thread-safe).
 * @param orientation New orientation as a quaternion.
 */
void HydraPlex::updateOrientation(Quaterniond orientation) {
    std::lock_guard<std::mutex> lock(dataMutex);
    this->orientation = orientation;
}

/**
 * Update the position offset (thread-safe).
 * @param posOffset New position offset.
 */
void HydraPlex::updatePosOffset(Vector3d posOffset) {
    std::lock_guard<std::mutex> lock(dataMutex);
    this->posOffset = posOffset;
}

/**
 * Update the velocity offset (thread-safe).
 * @param velOffset New velocity offset.
 */
void HydraPlex::updateVelOffset(Vector3d velOffset) {
    std::lock_guard<std::mutex> lock(dataMutex);
    this->velOffset = velOffset;
}

/**
 * Get the home point for a given motor, including its offset.
 * @param motor Index of the motor.
 * @return Home point in global coordinates.
 */
Vector3d HydraPlex::getHomePoint(uint8_t motor) {
    return homePoints[motor] + getOffset(motor);
}

/**
 * Get the offset vector for a given motor, rotated by the current orientation.
 * @param motor Index of the motor.
 * @return Rotated offset vector.
 */
Vector3d HydraPlex::getOffset(uint8_t motor) {
    std::lock_guard<std::mutex> lock(dataMutex);
    return qRotate(orientation, offsets[motor]);
}

/**
 * Trilaterate the node position using three motors.
 * @param indices Array of three motor indices.
 * @param side    Side of the solution (+1 or -1).
 * @return solutionType containing position and score.
 */
HydraPlex::solutionType HydraPlex::trilaterate(uint8_t* indices, int8_t side) {
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
    solution.score = exp(cbrt(radicand)); // Score based on solution quality (how far from baseline plane)
    return solution;
}

/**
 * Localize the node position using all motor combinations.
 * Uses weighted average of trilateration solutions.
 * @param stepTime Time step for velocity calculation (s).
 */
void HydraPlex::localize(double stepTime) {
    // Sum of position estimates
    Vector3d positionSum = Vector3d::Zero();
    double weightSum = 0;

    uint8_t combination[3] = {0, 1, 2};
    do
    {
        // For each combination of 3 motors, compute trilateration

        // Finds the correct side of the solution

        // Computes cross product of baseline vectors and compares to one home point vector
		// The assumption is that both the origin and the node position are on the positive side of the plane formed by the three points
        Vector3d v1, v2;
        v1 = getHomePoint(combination[1]) - getHomePoint(combination[0]);
        v2 = getHomePoint(combination[2]) - getHomePoint(combination[0]);
        double val = -v1.cross(v2).dot(getHomePoint(combination[0]));
        int8_t side = (int8_t)(val / abs(val));
        solutionType solution = trilaterate(combination, side);
        // Weights solutions by their score
        double weight = solution.score;
        positionSum += solution.position * weight;
        weightSum += weight;
    } while (nextCombination(NUM_MOTORS, 3, combination));

	// Finds weighted average position from all trilateration combinations
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
void HydraPlex::setForceTarget() {
    Vector3d force = Vector3d::Zero();
    setForceTarget(force);
}

/**
 * Set the force target for the node (thread-safe).
 * @param force Desired force vector.
 */
void HydraPlex::setForceTarget(Vector3d force) {
    std::lock_guard<std::mutex> lock(dataMutex);
    this->forceTarget = force;
}

/**
 * Disable collision-based position limiting.
 */
void HydraPlex::disableCollisionControl() {
    collisionEnabled = false;
}

/**
 * Set the collision target for the node (thread-safe).
 * Enables collision control and sets the collision point, normal, and time to collision.
 * @param collisionPoint   Point of collision.
 * @param collisionNormal  Normal vector at collision.
 * @param timeToCollision  Time until collision (s).
 */
void HydraPlex::setCollisionTarget(Vector3d collisionPoint, Vector3d collisionNormal, double timeToCollision) {
    std::lock_guard<std::mutex> lock(dataMutex);
    collisionEnabled = true;
    this->collisionPoint = collisionPoint;
    this->collisionNormal = collisionNormal;
    this->timeToCollision = timeToCollision;
}

/**
 * Update data for motors and sensors
 * Sets force targets and applies collision limits if enabled.
 */
void HydraPlex::updateData(double stepTime) {
    //Updates motors
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        motors[i].update();
    }
    
    // Updates thimble data
    thimble->update(stepTime);
}

/**
 * Update the controller for all motors.
 * Sets force targets and applies collision limits if enabled.
 */
void HydraPlex::updateController(double stepTime) {
    // Gets data from sensors
    Vector3d innerCapPos = thimble->getInnerCapPos();
    Quaterniond innerCapOrient = thimble->getInnerCapOrient();
    //Updates home point offsets based on IMU orientation
    updateOrientation(thimble->getOuterCapOrient());
    // Updates motor plex external position offset
    updatePosOffset(innerCapPos);
    updateVelOffset(thimble->getInnerCapVel());
    // Runs localization algorithm
    localize(stepTime);

    // Copies HydraPlex force target
    Vector3d forceTargetCopy;
    {
        std::lock_guard<std::mutex> lock(dataMutex);
        forceTargetCopy = forceTarget;
    }

    // If collision control is enabled, copy collision data
    Vector3d collisionPointCopy, collisionNormalCopy;
    double timeToCollisionCopy;
    if (collisionEnabled)
    {
        std::lock_guard<std::mutex> lock(dataMutex);
        collisionPointCopy = position + posOffset + collisionPoint;
        collisionNormalCopy = collisionNormal;
        timeToCollisionCopy = timeToCollision;
    }

    // Desired change in position of outer thimble
    // Currently set to follow inner thimble
    Vector3d targetChange = getPredPos() - getPredRawPos();

    // Avoids anticipated collision if within airgap range
    if (collisionEnabled) {
        double distToPlane = (getPredPos() - collisionPointCopy).dot(collisionNormalCopy);
        Vector3d vhat = getVelocity().normalized();
        Vector3d slant = -distToPlane / vhat.dot(collisionNormalCopy) * vhat;
        double overshoot = thimble->getAirGap() - slant.norm();
        if (overshoot > 0) {
            targetChange = targetChange - overshoot * slant.normalized();
        }
    }

    // Force correction for thimble following and collision anticipation
    Vector3d correctionForce = targetChange * controllerGain;

    if (forceTargetCopy.norm() == 0) {
        // If no target force is set, uses the pure correction
        forceTargetCopy = forceTargetCopy + correctionForce;
    }
    else {
        // Otherwise projects correction force onto an orthogonal plane
        Vector3d planeNormal = forceTargetCopy.normalized();
        double distToPlane = correctionForce.dot(planeNormal);
        // Adds projected force to target force
        forceTargetCopy = forceTargetCopy + correctionForce - distToPlane * planeNormal;
    }

    // Hydra motor force targets
    if (forceTargetCopy.norm() == 0) {
        // If no force is requested, set all motors to minimum force
        for (int i = 0; i < NUM_MOTORS; i++) {
            motors[i].setForceTarget(-minForce);
        }
    }
    else {
        // Compute string directions for each motor
        Matrix<double, 3, NUM_MOTORS> directions;
        for (int i = 0; i < NUM_MOTORS; i++) {
            directions.col(i) = (getPredRawPos() - getHomePoint(i)).normalized();
        }

        // Solve for force components for each string
        Vector<double, NUM_MOTORS> components = solveConstrainedForce(forceTargetCopy, directions);
        for (int i = 0; i < NUM_MOTORS; i++) {
            motors[i].setForceTarget(components(i));
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
Vector<double, NUM_MOTORS> HydraPlex::solveConstrainedForce(Vector3d forceTarget, Matrix<double, 3, NUM_MOTORS> directions) {
    // Finds particular solution using SVD
    JacobiSVD<MatrixXd> svd(directions, ComputeThinU | ComputeThinV);

    Vector<double, NUM_MOTORS> particular = svd.solve(forceTarget);

    // Finds null space of the directions matrix
    FullPivLU<MatrixXd> lu(directions);
    MatrixXd nullSpace = lu.kernel();
    Vector<double, NUM_MOTORS> nullBasis = nullSpace.col(0);

    // Computes intersections with all minimum force planes to find valid solution
    double minSum = 0;
    Vector<double, NUM_MOTORS> minSolution = Vector<double, NUM_MOTORS>::Zero();
    bool solutionFound = false;
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        if (nullBasis(i) != 0) {
            double intersection = (-minForce-particular(i)) / nullBasis(i);
            Vector<double, NUM_MOTORS> solution = particular + intersection * nullBasis;
            bool valid = true;
            for (int j = 0; j < NUM_MOTORS; j++) {
                valid &= (solution(j) <= -minForce+1e-5);
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
        // Default is all minForce
        components = -Eigen::VectorXd::Ones(NUM_MOTORS)*minForce;
    }
    return components;
}

/**
 * Get the current node position (thread-safe), not including position offset.
 * @return Node position vector.
 */
Vector3d HydraPlex::getRawPosition() {
    std::lock_guard<std::mutex> lock(dataMutex);
    return position;
}

/**
 * Get the current node position (thread-safe), including position offset.
 * @return Node position vector.
 */
Vector3d HydraPlex::getPosition() {
    std::lock_guard<std::mutex> lock(dataMutex);
    return position + posOffset;
}

/**
 * Get the predicted node position after horizon time (thread-safe), not including position offset.
 * @return Node position vector.
 */
Vector3d HydraPlex::getPredRawPos() {
    std::lock_guard<std::mutex> lock(dataMutex);
    return position + velocity*horizonTime;
}

/**
 * Get the predicted node position after horizon time (thread-safe), including position offset.
 * @return Node position vector.
 */
Vector3d HydraPlex::getPredPos() {
    std::lock_guard<std::mutex> lock(dataMutex);
    return position + posOffset + (velocity + velOffset) * horizonTime;
}

/**
 * Get the current node velocity (thread-safe), not including velocity offset.
 * @return Node velocity vector.
 */
Vector3d HydraPlex::getRawVelocity() {
    std::lock_guard<std::mutex> lock(dataMutex);
    return velocity;
}

/**
 * Get the current node velocity (thread-safe), including velocity offset.
 * @return Node velocity vector.
 */
Vector3d HydraPlex::getVelocity() {
    std::lock_guard<std::mutex> lock(dataMutex);
    return velocity + velOffset;
}

/**
 * Get the minimum force
 * @return minimum force
 */
double HydraPlex::getMinForce() {
    return minForce;
}

