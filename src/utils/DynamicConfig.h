/**
 * DyanmicConfig.h - Dynamic hardware peripheral configuration framework
 * Created by Carson G. Ray
 */

#ifndef DYNAMIC_CONFIG_H
#define DYNAMIC_CONFIG_H

 //External imports
#include <vector>
#include <string>
#include <mutex>
#include <map>
#include <fstream>

class DynamicConfig {
public:
    struct BusChainConfig {
        uint8_t bus; // I2C bus number (0 or 1)
        std::vector<uint8_t> moduleIds; // addresses of BusChain modules on bus
    };
    struct I2CDeviceConfig {
        bool onBusChain; // Whether sensor is on BusChain
        uint8_t busId; // buschain id (or bus if not on BusChain)
        uint8_t channel; // Channel on BusChain
    };
    struct IMUConfig : public I2CDeviceConfig {
        uint8_t accelMode;
        uint8_t gyroMode;
        uint8_t filterMode;
    };
    struct ServoConfig {
        uint8_t servoDriverId; // Servo driver id
        uint8_t channel; // Channel on servo driver
    };
    struct FOCMotorConfig {
        uint8_t port; // Built-in motor driver port (0 or 1)
    };

    void addBusChain(const BusChainConfig config);
    void addMagEncoder(const I2CDeviceConfig config);
    void addMagTracker(const I2CDeviceConfig config);
    void addIMU(const IMUConfig config);
    void addServoDriver(const I2CDeviceConfig config);
    void addServo(const ServoConfig config);
    void addFOCMotor(const FOCMotorConfig config);

    uint8_t numBusChains() const;
    uint8_t numMagEncoders() const;
    uint8_t numMagTrackers() const;
    uint8_t numIMUs() const;
    uint8_t numServos() const;
    uint8_t numServoDrivers() const;

    BusChainConfig getBusChain(uint8_t id) const;
    I2CDeviceConfig getMagEncoder(uint8_t id) const;
    I2CDeviceConfig getMagTracker(uint8_t id) const;
    IMUConfig getIMU(uint8_t id) const;
    I2CDeviceConfig getServoDriver(uint8_t id) const;
    ServoConfig getServo(uint8_t id) const;
    FOCMotorConfig getFOCMotor(uint8_t id) const;

    std::string describeBusChain(const BusChainConfig config) const;
    std::string describeI2CDevice(const I2CDeviceConfig config) const;

private:
    // Mutex for thread safety
    mutable std::mutex configMutex;

    // Vector of bus chain configurations
    std::vector<BusChainConfig> busChainConfigs;

    // Vectors for I2C device configurations
    std::vector<I2CDeviceConfig> magEncoderConfigs;
    std::vector<I2CDeviceConfig> magTrackerConfigs;
    std::vector<IMUConfig> imuConfigs;
    std::vector<I2CDeviceConfig> servoDriverConfigs;

    // Vector of servo configurations
    std::vector<ServoConfig> servoConfigs;

    // Vector of FOC motor configurations
    std::vector<FOCMotorConfig> focMotorConfigs;
};

#endif