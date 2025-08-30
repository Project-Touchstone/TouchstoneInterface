// Unit tests for HydraFOCMotor and HydraPlex
#include "gtest/gtest.h"
#include "../actuators/HydraFOCMotor.h"
#include "../actuators/HydraPlex.h"
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
    double relativePosition() override { return position - offset; }
    double absolutePosition() override { return position; }
    void reset() override { offset = position;}
};

TEST(HydraFOCMotorTest, AttachAndResetEncoder) {
    HydraFOCMotor motor;
    MockMagEncoder encoder;
    motor.attach(&encoder);
    encoder.setRelativePosition(5);
    motor.resetEncoder();
    // After reset, relativePosition should be 0
    EXPECT_EQ(encoder.relativePosition(), 0);
}

TEST(HydraFOCMotorTest, SetForceTarget) {
    HydraFOCMotor motor;
    MockMagEncoder encoder;
    motor.attach(&encoder);
	motor.setMotorDir(-1); // Set motor direction to -1
    motor.setForceTarget(0.5);
    EXPECT_EQ(motor.getMode(), HydraFOCMotor::FORCE);
    EXPECT_NEAR(motor.getTorqueTarget(), 0.5*HydraFOCMotor::getSpoolRadius(), 1e-6);
	motor.setMotorDir(1); // Reset motor direction to 1
    motor.setForceTarget(0.5);
	EXPECT_NEAR(motor.getTorqueTarget(), 0.5 * HydraFOCMotor::getSpoolRadius(), 1e-6);
}

TEST(HydraFOCMotorTest, SetVelocityTarget) {
    HydraFOCMotor motor;
    MockMagEncoder encoder;
    motor.attach(&encoder);
    motor.setMotorDir(-1); // Set motor direction to -1
    motor.setVelocityTarget(0.5);
    EXPECT_EQ(motor.getMode(), HydraFOCMotor::VELOCITY);
    EXPECT_NEAR(motor.getOmegaTarget(), 0.5 / HydraFOCMotor::getSpoolRadius(), 1e-6);
    motor.setMotorDir(1); // Reset motor direction to 1
    motor.setVelocityTarget(0.5);
    EXPECT_NEAR(motor.getOmegaTarget(), 0.5 / HydraFOCMotor::getSpoolRadius(), 1e-6);
}

TEST(HydraFOCMotorTest, SetPositionTarget) {
    HydraFOCMotor motor;
    MockMagEncoder encoder;
    motor.attach(&encoder);
    motor.setMotorDir(-1); // Set motor direction to -1
    motor.setPositionTarget(0.5);
    EXPECT_EQ(motor.getMode(), HydraFOCMotor::POSITION);
    EXPECT_NEAR(motor.getPositionTarget(), 0.5 / HydraFOCMotor::getSpoolRadius(), 1e-6);
    motor.setMotorDir(1); // Reset motor direction to 1
    motor.setPositionTarget(0.5);
    EXPECT_NEAR(motor.getPositionTarget(), 0.5 / HydraFOCMotor::getSpoolRadius(), 1e-6);
}

TEST(HydraFOCMotorTest, EncoderPos) {
    HydraFOCMotor motor;
    MockMagEncoder encoder;
    motor.attach(&encoder);
    encoder.setRelativePosition(1.0);

    EXPECT_NEAR(motor.getEncoderPos(), 1.0, 1e-6);
}

TEST(HydraFOCMotorTest, GetPosition) {
    HydraFOCMotor motor;
    MockMagEncoder encoder;
    motor.attach(&encoder);
    encoder.setRelativePosition(1.0);
    
    EXPECT_NEAR(motor.getPosition(), 1.0 * HydraFOCMotor::getSpoolRadius(), 1e-6);
}

TEST(HydraFOCMotorTest, HomingBehavior) {
    HydraFOCMotor motor;
    MockMagEncoder encoder;
    motor.attach(&encoder);
    // Set initial position
    encoder.setRelativePosition(3.0);
    // Begin homing
    motor.beginHoming();
    EXPECT_TRUE(motor.isHoming());
    // Simulate spool moving below homePos
    encoder.setRelativePosition(-2.0);
    // Call updateMPC to trigger homePos update
    motor.update();
    // After updateMPC, homePos should be updated to -2.0, so getPosition() should be zero
    EXPECT_NEAR(motor.getPosition(), 0.0, 1e-6);
    // End homing
    motor.endHoming();
    EXPECT_FALSE(motor.isHoming());
}

TEST(HydraPlexTest, AttachAndGetHomePoint) {
    HydraFOCMotor motors[4];
    Thimble thimble;
    Vector3d homePoints[4];
    Vector3d offsets[4];
    for (int i = 0; i < 4; ++i) {
        homePoints[i] = Vector3d(i, i + 1, i + 2);
        offsets[i] = Vector3d(1, 0, 0);
    }
    HydraPlex plex;
    plex.attach(motors, &thimble, homePoints, offsets);
    auto hp = plex.getHomePoint(0);
    EXPECT_TRUE(hp.isApprox(Vector3d(1, 1, 2)));
}

TEST(HydraPlexTest, GetPositionAndVelocity) {
    HydraPlex plex;
    HydraFOCMotor motors[4];
    Thimble thimble;
    Vector3d homePoints[4];
    Vector3d offsets[4];
    plex.attach(motors, &thimble, homePoints, offsets);
    plex.updatePosOffset(Vector3d(1, 2, 3));
    plex.updateVelOffset(Vector3d(4, 5, 6));
    // Should return default values
    EXPECT_TRUE(plex.getPosition().isApprox(Vector3d(1, 2, 3)));
    EXPECT_TRUE(plex.getVelocity().isApprox(Vector3d(4, 5, 6)));
}

TEST(HydraPlexTest, Localization) {
    HydraPlex plex;
    HydraFOCMotor motors[4];
    MockMagEncoder encoders[4];
    Thimble thimble;
    Vector3d homePoints[4] = { Vector3d(0, 0, -1), Vector3d(-1, 0, 1), Vector3d(0, -1, 1), Vector3d(1, 1, 1) };
    Vector3d offsets[4] = { Vector3d(0.1, 0.1, 0), Vector3d(0.1, -0.1, 0), Vector3d(-0.1, 0.1, 0), Vector3d(-0.1, -0.1, 0) };
    // Test node position
    Vector3d testNodePos = Vector3d(1, 1, 1);
    // Attach each motor to its own mock encoder
    for (int i = 0; i < 4; ++i) {
        motors[i].attach(&encoders[i]);
    }
    plex.attach(motors, &thimble, homePoints, offsets);
    // Set encoder positions to string distances from origin to homePoint+offset
    for (uint8_t i = 0; i < 4; ++i) {
        Vector3d anchor = (homePoints[i] + offsets[i])-testNodePos;
        double dist = anchor.norm();
        encoders[i].setRelativePosition(-dist / HydraFOCMotor::getSpoolRadius()); // spool encoder (note encoder dir is -1)
    }
    // Updates the position of the node based on trilateration
    plex.localize(0.01);
    // The expected position is close test position
    EXPECT_TRUE(plex.getPosition().isApprox(testNodePos));
    // The expected velocity is the difference in displacement over the step time
    EXPECT_TRUE(plex.getVelocity().isApprox(testNodePos / 0.01));
}

TEST(HydraPlexTest, SolveConstrainedForce) {
    HydraPlex plex;
    Vector3d homePoints[4] = { Vector3d(0, 0, -1), Vector3d(-1, 0, 1), Vector3d(0, -1, 1), Vector3d(1, 1, 1) };
    // Mock force target and directions
    Vector3d forceTarget(10, 10, 10);
    Matrix<double, 3, 4> directions;
    for (uint8_t i = 0; i < 4; ++i) {
        directions.col(i) = (homePoints[i] - Vector3d::Zero()).normalized();
    }
    auto components = plex.solveConstrainedForce(forceTarget, directions);
    // Check that components vector is the right size
    EXPECT_EQ(components.size(), 4);
    // Check that components are all greater magnetiude than minForce
    for (uint8_t i = 0; i < 4; i++) {
        EXPECT_TRUE(components[i] < -HydraPlex::getMinForce());
    }
    // Check that the sum of the force components in the directions is close to the force target
    Vector3d forceSum = directions * components;
    EXPECT_TRUE(forceSum.isApprox(forceTarget), 0, 1e-6);
}