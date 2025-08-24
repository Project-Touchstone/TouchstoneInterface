#ifndef HardwareConfigMap_h
#define HardwareConfigMap_h

#include "../utils/DynamicConfig.h"

DynamicConfig::BusChainConfig busChainConfigs[] = {
    {0, {0}},
    {1, {1}}
};

DynamicConfig::I2CDeviceConfig magEncoderConfigs[] = {
    {false, 0, 0},
    {false, 1, 0}
};

DynamicConfig::I2CDeviceConfig magTrackerConfigs[] = {
    {true, 0, 0},
    {true, 0, 1}
};

DynamicConfig::IMUConfig imuConfigs[] = {
    {true, 0, 0},
    {true, 0, 1}
};

DynamicConfig::I2CDeviceConfig servoDriverConfigs[] = {

};

DynamicConfig::ServoConfig servoConfigs[] = {

};

DynamicConfig::FOCMotorConfig focMotorConfigs[] = {

};

void loadConfig(DynamicConfig& config) {
  // Add BusChain configs
  for (const auto& bc : busChainConfigs) {
    config.addBusChain(bc);
  }
  // Add magnetic encoder configs
  for (const auto& me : magEncoderConfigs) {
    config.addMagEncoder(me);
  }
  // Add magnetic tracker configs
  for (const auto& mt : magTrackerConfigs) {
    config.addMagTracker(mt);
  }
  // Add IMU configs
  for (const auto& imu : imuConfigs) {
    config.addIMU(imu);
  }
  // Add servo driver configs
  for (const auto& servoDriver : servoDriverConfigs) {
    config.addServoDriver(servoDriver);
  }
  // Add servo configs
  for (const auto& servo : servoConfigs) {
    config.addServo(servo);
  }
  // Add foc motor configs
  for (const auto focMotor : focMotorConfigs) {
    config.addFOCMotor(focMotor);
  }
};

#endif