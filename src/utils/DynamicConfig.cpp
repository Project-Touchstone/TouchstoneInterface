/**
 * DyanmicConfig.cpp - Dynamic hardware peripheral configuration framework
 * Created by Carson G. Ray
 */

#include "DynamicConfig.h"

std::size_t DynamicConfig::addBusChain(const BusChainConfig config) {
    std::lock_guard<std::mutex> lock(configMutex);
    busChainConfigs.push_back(config);
    return busChainConfigs.size() - 1;
}

std::size_t DynamicConfig::addMagEncoder(const I2CDeviceConfig config) {
    std::lock_guard<std::mutex> lock(configMutex);
    magEncoderConfigs.push_back(config);
    return magEncoderConfigs.size() - 1;
}

std::size_t DynamicConfig::addMagTracker(const I2CDeviceConfig config) {
    std::lock_guard<std::mutex> lock(configMutex);
    magTrackerConfigs.push_back(config);
    return magTrackerConfigs.size() - 1;
}

std::size_t DynamicConfig::addIMU(const IMUConfig config) {
    std::lock_guard<std::mutex> lock(configMutex);
    imuConfigs.push_back(config);
    return imuConfigs.size() - 1;
}

std::size_t DynamicConfig::addServoDriver(const I2CDeviceConfig config) {
    std::lock_guard<std::mutex> lock(configMutex);
    servoDriverConfigs.push_back(config);
    return servoDriverConfigs.size() - 1;
}

std::size_t DynamicConfig::addServo(const ServoConfig config) {
    std::lock_guard<std::mutex> lock(configMutex);
    servoConfigs.push_back(config);
    return servoConfigs.size() - 1;
}

std::size_t DynamicConfig::addFOCMotor(const FOCMotorConfig config) {
    std::lock_guard<std::mutex> lock(configMutex);
    focMotorConfigs.push_back(config);
    return focMotorConfigs.size() - 1;
}

uint8_t DynamicConfig::numBusChains() const {
    std::lock_guard<std::mutex> lock(configMutex);
    return busChainConfigs.size();
}

uint8_t DynamicConfig::numMagEncoders() const {
    std::lock_guard<std::mutex> lock(configMutex);
    return magEncoderConfigs.size();
}

uint8_t DynamicConfig::numMagTrackers() const {
    std::lock_guard<std::mutex> lock(configMutex);
    return magTrackerConfigs.size();
}

uint8_t DynamicConfig::numIMUs() const {
    std::lock_guard<std::mutex> lock(configMutex);
    return imuConfigs.size();
}

uint8_t DynamicConfig::numServos() const {
    std::lock_guard<std::mutex> lock(configMutex);
    return servoConfigs.size();
}

uint8_t DynamicConfig::numServoDrivers() const {
    std::lock_guard<std::mutex> lock(configMutex);
    return servoDriverConfigs.size();
}

uint8_t DynamicConfig::numFOCMotors() const {
    std::lock_guard<std::mutex> lock(configMutex);
    return focMotorConfigs.size();
}

// Length of sensor data packet
uint8_t DynamicConfig::getSensorDataLength() const {
    return magEncoderLen * numMagEncoders() + magTrackerLen * numMagTrackers() + imuLen * numIMUs();
}

DynamicConfig::BusChainConfig DynamicConfig::getBusChain(uint8_t id) const {
    std::lock_guard<std::mutex> lock(configMutex);
    if (id < busChainConfigs.size()) {
        return busChainConfigs[id];
    }
    return BusChainConfig{};
}

DynamicConfig::I2CDeviceConfig DynamicConfig::getMagEncoder(uint8_t id) const {
    std::lock_guard<std::mutex> lock(configMutex);
    if (id < magEncoderConfigs.size()) {
        return magEncoderConfigs[id];
    }
    return I2CDeviceConfig{};
}

DynamicConfig::I2CDeviceConfig DynamicConfig::getMagTracker(uint8_t id) const {
    std::lock_guard<std::mutex> lock(configMutex);
    if (id < magTrackerConfigs.size()) {
        return magTrackerConfigs[id];
    }
    return I2CDeviceConfig{};
}

DynamicConfig::IMUConfig DynamicConfig::getIMU(uint8_t id) const {
    std::lock_guard<std::mutex> lock(configMutex);
    if (id < imuConfigs.size()) {
        return imuConfigs[id];
    }
    return IMUConfig{};
}

DynamicConfig::I2CDeviceConfig DynamicConfig::getServoDriver(uint8_t id) const {
    std::lock_guard<std::mutex> lock(configMutex);
    if (id < servoDriverConfigs.size()) {
        return servoDriverConfigs[id];
    }
    return I2CDeviceConfig{};
}

DynamicConfig::ServoConfig DynamicConfig::getServo(uint8_t id) const {
    std::lock_guard<std::mutex> lock(configMutex);
    if (id < servoConfigs.size()) {
        return servoConfigs[id];
    }
    return ServoConfig{};
}

DynamicConfig::FOCMotorConfig DynamicConfig::getFOCMotor(uint8_t id) const {
    std::lock_guard<std::mutex> lock(configMutex);
    if (id < focMotorConfigs.size()) {
        return focMotorConfigs[id];
    }
    return FOCMotorConfig{};
}

void DynamicConfig::beginIMU(uint8_t id, IMU& imu) {
    DynamicConfig::IMUConfig config = getIMU(id);
    imu.setRanges(imuAccelRanges[config.accelMode], imuGyroRanges[config.gyroMode]);
}

std::string DynamicConfig::describeBusChain(const BusChainConfig config) const {
    std::lock_guard<std::mutex> lock(configMutex);
    std::string result = " on bus ";
    result = result + std::to_string(config.bus) + " with modules ";
    for (uint8_t i = 0; i < config.moduleIds.size(); i++) {
        result = result + std::to_string(config.moduleIds[i]);
        if (i < config.moduleIds.size() - 1) {
            result = result + ", ";
        }
    }
    return result;
}

std::string DynamicConfig::describeI2CDevice(const I2CDeviceConfig config) const {
    std::lock_guard<std::mutex> lock(configMutex);
    std::string busChainStr;
    std::string channelStr;
    if (config.onBusChain) {
        busChainStr = "on BusChain ";
        channelStr = " channel " + std::to_string(config.channel);
    }
    else {
        busChainStr = "on direct bus ";
        channelStr = "";
    }
    return busChainStr + std::to_string(config.busId) + channelStr;
}

std::string DynamicConfig::describeServo(const ServoConfig config) const {
    std::lock_guard<std::mutex> lock(configMutex);
    return "on servo driver " + std::to_string(config.servoDriverId) + " channel " + std::to_string(config.channel);
}

std::string DynamicConfig::describeFOCMotor(const FOCMotorConfig config) const {
    std::lock_guard<std::mutex> lock(configMutex);
    return "on port " + std::to_string(config.port);
}