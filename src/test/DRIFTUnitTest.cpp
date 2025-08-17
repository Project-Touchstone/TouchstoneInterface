// Unit tests for DRIFTMotor and DRIFTPlex
#include "gtest/gtest.h"
#include "../actuators/DRIFTMotor.h"
#include "../actuators/DRIFTPlex.h"
#include "../sensors/MagEncoder.h"
#include <Eigen/Dense>
#include <array>

using namespace Eigen;

// Mock MagEncoder for deterministic testing
class MockMagEncoder : public MagEncoder {
public:
    double position = 0;
    double offset = 0;
    double velocity = 0;
    void setRelativePosition(double v) { position = v + offset; }
    void setAbsolutePosition(double v) { position = v; }
    void setSampledVelocity(double v) { velocity = v; }
    double relativePosition() override { return position - offset; }
    double absolutePosition() override { return position; }
    double sampledVelocity() override { return velocity; }
    void reset() override { offset = position;}
};

TEST(DRIFTMotorTest, AttachAndResetEncoders) {
    DRIFTMotor motor;
    MockMagEncoder servo, spool;
    motor.attach(&servo, &spool);
    servo.setRelativePosition(5);
    spool.setRelativePosition(10);
    motor.resetEncoders();
    // After reset, relativePosition should be 0
    EXPECT_EQ(servo.relativePosition(), 0);
    EXPECT_EQ(spool.relativePosition(), 0);
}

TEST(DRIFTMotorTest, SetAndGetPower) {
    DRIFTMotor motor;
    MockMagEncoder servo, spool;
    motor.attach(&servo, &spool);
	motor.setMotorDir(-1); // Set motor direction to -1
    motor.setPower(0.5);
    EXPECT_NEAR(motor.getPower(), -0.5, 1e-6);
	motor.setMotorDir(1); // Reset motor direction to 1
    motor.setPower(0.5);
	EXPECT_NEAR(motor.getPower(), 0.5, 1e-6);
}

TEST(DRIFTMotorTest, SetForceTarget) {
    DRIFTMotor motor;
    MockMagEncoder servo, spool;
    motor.attach(&servo, &spool);
    motor.setForceTarget(-2.0);
    // Should set mode to FORCE and separationTarget > minSep
    EXPECT_EQ(motor.getMode(), DRIFTMotor::FORCE);
}

TEST(DRIFTMotorTest, HomingState) {
    DRIFTMotor motor;
    MockMagEncoder servo, spool;
    motor.attach(&servo, &spool);
    motor.beginHoming();
    EXPECT_TRUE(motor.isHoming());
    motor.endHoming();
    EXPECT_FALSE(motor.isHoming());
}

TEST(DRIFTMotorTest, GetPositionAndVelocity) {
    DRIFTMotor motor;
    MockMagEncoder servo, spool;
    motor.attach(&servo, &spool);
    servo.setRelativePosition(1.0);
    spool.setRelativePosition(2.0);
    servo.setSampledVelocity(0.1);
    spool.setSampledVelocity(0.2);
    motor.sampleVelocity();
    
    // getPosition uses spool encoder
    EXPECT_NEAR(motor.getPosition(), (2.0 - 0.0) * DRIFTMotor::getUnitsPerRadian(), 1e-6);
    EXPECT_NEAR(motor.getVelocity(), 0.2 * DRIFTMotor::getUnitsPerRadian(), 1e-6);
}

TEST(DRIFTPlexTest, AttachAndGetHomePoint) {
    DRIFTMotor motors[NUM_MOTORS];
    Vector3d homePoints[NUM_MOTORS];
    Vector3d offsets[NUM_MOTORS];
    for (int i = 0; i < NUM_MOTORS; ++i) {
        homePoints[i] = Vector3d(i, i+1, i+2);
        offsets[i] = Vector3d(1, 0, 0);
    }
    DRIFTPlex plex;
    plex.attach(motors, homePoints, offsets);
    auto hp = plex.getHomePoint(0);
    EXPECT_TRUE(hp.isApprox(Vector3d(1,1,2)));
}

TEST(DRIFTPlexTest, SetForceTargetAndDisableCollision) {
    DRIFTPlex plex;
    DRIFTMotor motors[NUM_MOTORS];
    Vector3d homePoints[NUM_MOTORS];
    Vector3d offsets[NUM_MOTORS];
    plex.attach(motors, homePoints, offsets);
    Vector3d force(1,2,3);
    plex.setForceTarget(force);
    plex.disableCollisionControl();
    // No crash, state set
}

TEST(DRIFTPlexTest, UpdateOrientationAndOffsets) {
    DRIFTPlex plex;
    DRIFTMotor motors[NUM_MOTORS];
    Vector3d homePoints[NUM_MOTORS];
    Vector3d offsets[NUM_MOTORS];
    plex.attach(motors, homePoints, offsets);
    Quaterniond q = Quaterniond::Identity();
    plex.updateOrientation(q);
    plex.updatePosOffset(Vector3d(1,2,3));
    plex.updateVelOffset(Vector3d(4,5,6));
    // No crash, state set
}

TEST(DRIFTPlexTest, GetPositionAndVelocity) {
    DRIFTPlex plex;
    DRIFTMotor motors[NUM_MOTORS];
    Vector3d homePoints[NUM_MOTORS];
    Vector3d offsets[NUM_MOTORS];
    plex.attach(motors, homePoints, offsets);
    // Should return default values
    EXPECT_TRUE(plex.getPosition().isApprox(Vector3d::Zero()));
    EXPECT_TRUE(plex.getVelocity().isApprox(Vector3d::Zero()));
}

TEST(DRIFTMotorTest, HomingBehavior) {
    DRIFTMotor motor;
    MockMagEncoder servo, spool;
    motor.attach(&servo, &spool);
    // Set initial positions
    servo.setRelativePosition(0.0);
    spool.setRelativePosition(5.0);
    // Begin homing
    motor.beginHoming();
    EXPECT_TRUE(motor.isHoming());
    // Simulate spool moving below homePos
    spool.setRelativePosition(-2.0);
    // Call updateMPC to trigger homePos update
    motor.updateMPC();
    // After updateMPC, homePos should be updated to -2.0, so getPosition() should be zero
    EXPECT_NEAR(motor.getPosition(), 0.0, 1e-6);
    // End homing
    motor.endHoming();
    EXPECT_FALSE(motor.isHoming());
}

TEST(DRIFTMotorTest, ManualModeSetPower) {
    DRIFTMotor motor;
    MockMagEncoder servo, spool;
    motor.attach(&servo, &spool);
    motor.setPower(0.7);
    EXPECT_EQ(motor.getMode(), DRIFTMotor::MANUAL);
    EXPECT_NEAR(motor.getPower(), -0.7, 1e-6);
}

TEST(DRIFTMotorTest, ForceModeSetForceTarget) {
    DRIFTMotor motor;
    MockMagEncoder servo, spool;
    motor.attach(&servo, &spool);
    motor.setForceTarget(-3.0);
    EXPECT_EQ(motor.getMode(), DRIFTMotor::FORCE);
    // Setting force to zero should also set mode to FORCE
    motor.setForceTarget(0.0);
    EXPECT_EQ(motor.getMode(), DRIFTMotor::FORCE);
}

TEST(DRIFTMotorTest, PositionModeSetPositionLimit) {
    DRIFTMotor motor;
    MockMagEncoder servo, spool;
    motor.attach(&servo, &spool);
    double target = 10.0;
    motor.setPositionLimit(target);
    EXPECT_EQ(motor.getMode(), DRIFTMotor::POSITION);
}

TEST(DRIFTMotorTest, SeparationCalculation) {
    DRIFTMotor motor;
    MockMagEncoder servo, spool;
    motor.attach(&servo, &spool);
    servo.setRelativePosition(2.0);
    spool.setRelativePosition(5.0);
    EXPECT_NEAR(motor.getSeparation(), (5.0 - 2.0) * DRIFTMotor::getUnitsPerRadian(), 1e-6);
}

TEST(DRIFTMotorTest, EncoderPosAndVel) {
    DRIFTMotor motor;
    MockMagEncoder servo, spool;
    motor.attach(&servo, &spool);
    servo.setRelativePosition(3.0);
    spool.setRelativePosition(7.0);
    servo.setSampledVelocity(0.5);
    spool.setSampledVelocity(1.5);
    motor.sampleVelocity();
    EXPECT_NEAR(motor.getEncoderPos(0), 3.0, 1e-6);
    EXPECT_NEAR(motor.getEncoderPos(1), 7.0, 1e-6);
    EXPECT_NEAR(motor.getEncoderVel(0), 0.5, 1e-6);
    EXPECT_NEAR(motor.getEncoderVel(1), 1.5, 1e-6);
}

TEST(DRIFTPlexTest, Localization) {
    DRIFTPlex plex;
    DRIFTMotor motors[NUM_MOTORS];
    MockMagEncoder encoders[NUM_MOTORS][2];
    Vector3d homePoints[NUM_MOTORS] = { Vector3d(0, 0, -1), Vector3d(-1, 0, 1), Vector3d(0, -1, 1), Vector3d(1, 1, 1) };
    Vector3d offsets[NUM_MOTORS] = { Vector3d(0.1, 0.1, 0), Vector3d(0.1, -0.1, 0), Vector3d(-0.1, 0.1, 0), Vector3d(-0.1, -0.1, 0) };
    // Attach each motor to its own pair of mock encoders
    for (int i = 0; i < NUM_MOTORS; ++i) {
        motors[i].attach(&encoders[i][0], &encoders[i][1]);
    }
    plex.attach(motors, homePoints, offsets);
    // Set encoder positions to string distances from origin to homePoint+offset
    for (int i = 0; i < NUM_MOTORS; ++i) {
        Vector3d anchor = homePoints[i] + offsets[i];
        double dist = anchor.norm();
        encoders[i][0].setRelativePosition(0.0); // servo encoder (not used in trilaterate)
        encoders[i][1].setRelativePosition(-dist / DRIFTMotor::getUnitsPerRadian()); // spool encoder (note encoder dir is -1)
    }
    // Updates the position of the node based on trilateration
    plex.localize(0.01);
    // The expected position is close to the origin (since initial node point is at the orgin)
    EXPECT_NEAR(plex.getPosition().norm(), 0, 1e-3);
    
}

TEST(DRIFTPlexTest, SolveConstrainedForce) {
    DRIFTPlex plex;
    Vector3d homePoints[NUM_MOTORS] = { Vector3d(0, 0, -1), Vector3d(-1, 0, 1), Vector3d(0, -1, 1), Vector3d(1, 1, 1) };
    // Mock force target and directions
    Vector3d forceTarget(10, 10, 10);
    Matrix<double, 3, NUM_MOTORS> directions;
    for (int i = 0; i < NUM_MOTORS; ++i) {
        directions.col(i) = (homePoints[i] - Vector3d::Zero()).normalized();
    }
    auto components = plex.solveConstrainedForce(forceTarget, directions);
    EXPECT_EQ(components.size(), NUM_MOTORS);
    // Check that the sum of the force components in the directions is close to the force target
    Vector3d forceSum = directions * components;
    EXPECT_NEAR((forceSum - forceTarget).norm(), 0, 1e-6);
}