// TouchstoneInterface.cpp : Defines the entry point for the application.

#include "TouchstoneInterface.h"

using namespace FirmwareHeaders;
using namespace ApplicationHeaders;
using namespace Eigen;
using namespace boost;
using namespace Utils;

//Firmware serial interface
MinBiTSerialNode firmware("Firmware Interface");

//Firmware protocol
std::shared_ptr<MinBiTCore> firmwareData;

//Application layer TCP/IP interface
MinBiTTcpServer application("Application Interface", SERVER_PORT);

//Application protocol
std::shared_ptr<MinBiTCore> appData;

//Dynamic configuration object
DynamicConfig config;

// Encoder objects
MagEncoder magEncoders[NUM_MOTORS];

//Magnetic tracker objects
MagTracker magTrackers[2];

//Thimble object
Thimble thimble;

//IMU objects
IMU imus[1];

HydraPlex motorPlex;
HydraFOCMotor motors[NUM_MOTORS];

Servo servos[1];

const uint16_t calibrationTime[2] = { 3000, 500};

Vector3d homePoints[NUM_MOTORS];
Vector3d offsets[NUM_MOTORS];

//True orientation of thimble
Quaterniond trueOrient = Quaterniond::Identity();

//Homing power
const double homingPower = 1;
//Homing time
const uint16_t homingTime[2] = { 10000, 5000 };

//Finger cap parameters
double capRadius = 0.018822;
double capHeight = 0.03025;

//Servo power multiplier
float servoPowerSerialize = 32767;

// Phase completion flags

// Whether connection with hardware is alive
bool aliveFlag = false;

// Whether hardware has been successfully configured
bool configFlag = false;

// Whether sensors and actuators have been calibrated
bool calibrationFlag = false;

// Whether device has been homed
bool homeFlag = false;

// Processing notification
std::condition_variable processCondition;
std::mutex processMutex;

//DEBUG: Time between processing cycles
Timer processTimer;

//DEBUG: Time between serial cycles
Timer printTimer;


uint8_t setup() {
    //Initializes DRIFT motor outlet points (x, y, z)
    homePoints[0] = { 0, 0, -0.124404 };
    homePoints[1] = { -0.136127, -0.061985, 0.124404 };
    homePoints[2] = { 0.012252, 0.152579, 0.124404 };
    homePoints[3] = { 0.123881, -0.083203, 0.124404 };

    offsets[0] = { 0, capHeight / 2, capRadius };
    offsets[1] = { (double)(capRadius * cos(EIGEN_PI / 6)), capHeight / 2, (double)(-capRadius * sin(EIGEN_PI / 6)) };
    offsets[2] = { 0, -capHeight / 2, 0 };
    offsets[3] = { (double)(-capRadius * cos(EIGEN_PI / 6)), capHeight / 2, (double)(-capRadius * sin(EIGEN_PI / 6)) };

    //Attaches encoders to motors
    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].attach(&magEncoders[i]);
    }
    //Gives homing points and motors to DRIFTPlex
    motorPlex.attach(motors, &thimble, homePoints, offsets);

    //Attaches magnetic trackers to thimble object
    thimble.attach(magTrackers, &imus[0]);

    //Sets tracker orientations
    magTrackers[0].setSensorOrientation(eulerToQuat(Vector3d(-EIGEN_PI / 2, -EIGEN_PI, 0)));
    magTrackers[1].setSensorOrientation(eulerToQuat(Vector3d(EIGEN_PI / 2, 0, 0)));

    //Sets tracker positions
    magTrackers[0].setInitialPosition(Vector3d(0, 0, 1));
    magTrackers[1].setInitialPosition(Vector3d(0, 0, 1));

    // Sets IMU orientation offset
    imus[0].setOrientationOffset(eulerToQuat(Vector3d(-EIGEN_PI / 2, 0, EIGEN_PI / 2)));

    // Sets firmware data handler
    firmware.setReadHandler(&firmwareReadHandler);
    // Gets firmware data protocol
    firmwareData = firmware.getProtocol();
    // Loads protocol info
    firmwareData->loadPacketLengthsFromJson(FIRMWARE_PACKET_CONFIG);

    // If connection fails, return the error code otherwise, display a success message
    if (!firmware.begin(SERIAL_PORT, BAUD_RATE)) return 1;
    printf("Successful connection to %s\n", SERIAL_PORT);

    //Creates application request handler
    application.setReadHandler(&appReadHandler);
    // Gets application data protocol
    appData = application.getProtocol();
    // Loads protocol info
    appData->loadPacketLengthsFromJson(APPLICATION_PACKET_CONFIG);

    //Starts application interface
    application.begin();

    //Loads configuration mapping
    loadConfig(config);

    return 0;
}

int main()
{
    //Initializes parameters
    uint8_t error = setup();
    if (error > 0) {
        return error;
    }

    // Waits a bit so that there is no communication interface with reboot
    sleep(1000);

    // Begins threads
    thread schedulerT(schedulerThread);
    thread processingT(processingThread);

    schedulerT.join();
    processingT.join();

    // Cleanup
    firmware.end();
	application.end();
    return 0;
}
/*--------------------------------------------------*/
/*---------------------- Threads ---------------------*/
/*--------------------------------------------------*/
void schedulerThread() {
    // Pings microcontroller
    firmwareData->writeRequest(PING);
    firmwareData->sendAll();
    // Waits for serial connection to become live
    while (!aliveFlag) {
        sleep(10);
    }
    std::cout << "Configuration phase" << std::endl;
    if (!configuration()) {
        std::cout << "Configuration failed" << std::endl;
        return;
    }
    std::cout << "Calibration phase" << std::endl;
    calibration();
    processTimer.reset();
    std::cout << "Homing phase" << std::endl;
    homing();
    std::cout << "Homing complete" << std::endl;
}

bool configuration() {
    // Configures BusChains
    for (uint8_t i = 0; i < config.numBusChains(); i++) {
        DynamicConfig::BusChainConfig bcConfig = config.getBusChain(i);
        Request request = firmwareData->writeRequest(CONFIG_BUSCHAIN);

        // Length byte
        std::size_t modules = bcConfig.moduleIds.size();
        firmwareData->writeByte(modules + 1);
        // I2C bus
        firmwareData->writeByte(bcConfig.bus);
        // Module ids
        for (uint8_t j = 0; j < modules; j++) {
            firmwareData->writeByte(bcConfig.moduleIds[j]);
        }
        firmwareData->sendAll();
        request->WaitAsync().get();
        if (request->IsTimedOut() || request->GetResponseHeader() == NACK) {
            std::cout << "BusChain configuration failed: " +  config.describeBusChain(bcConfig) << std::endl;
            return false;
        }
        else {
            firmwareData->clearRequest();
        }
    }

    // Configures magnetic encoders
    for (uint8_t i = 0; i < config.numMagEncoders(); i++) {
        DynamicConfig::I2CDeviceConfig i2cConfig = config.getMagEncoder(i);
        Request request = firmwareData->writeRequest(CONFIG_BUSCHAIN + i2cConfig.onBusChain);

        // I2C bus or BusChain id
        firmwareData->writeByte(i2cConfig.busId);
        // BusChain channel
        if (i2cConfig.onBusChain) {
            firmwareData->writeByte(i2cConfig.channel);
        }
        firmwareData->sendAll();
        request->WaitAsync().get();
        if (request->IsTimedOut() || request->GetResponseHeader() == NACK) {
            std::cout << "Magnetic encoder configuration failed: " + config.describeI2CDevice(i2cConfig) << std::endl;
            return false;
        }
        else {
            firmwareData->clearRequest();
        }
    }

    // Configures magnetic trackers
    for (uint8_t i = 0; i < config.numMagTrackers(); i++) {
        DynamicConfig::I2CDeviceConfig i2cConfig = config.getMagTracker(i);
        Request request = firmwareData->writeRequest(CONFIG_MAG_TRACKER + i2cConfig.onBusChain);

        // I2C bus or BusChain id
        firmwareData->writeByte(i2cConfig.busId);
        // BusChain channel
        if (i2cConfig.onBusChain) {
            firmwareData->writeByte(i2cConfig.channel);
        }
        firmwareData->sendAll();
        request->WaitAsync().get();
        if (request->IsTimedOut() || request->GetResponseHeader() == NACK) {
            std::cout << "Magnetic tracker configuration failed: " + config.describeI2CDevice(i2cConfig) << std::endl;
            return false;
        }
        else {
            firmwareData->clearRequest();
        }
    }

    // Configures imus
    for (uint8_t i = 0; i < config.numIMUs(); i++) {
        DynamicConfig::IMUConfig imuConfig = config.getIMU(i);
        // Sets imu object parameters based on configuration
        config.beginIMU(i, imus[i]);
        
        // Sends imu configuration data
        Request request = firmwareData->writeRequest(CONFIG_IMU + imuConfig.onBusChain);

        // I2C bus or BusChain id
        firmwareData->writeByte(imuConfig.busId);
        // BusChain channel
        if (imuConfig.onBusChain) {
            firmwareData->writeByte(imuConfig.channel);
        }
        // IMU parameters
        firmwareData->writeByte(imuConfig.accelMode);
        firmwareData->writeByte(imuConfig.gyroMode);
        firmwareData->writeByte(imuConfig.filterMode);
        firmwareData->sendAll();
        request->WaitAsync().get();
        if (request->IsTimedOut() || request->GetResponseHeader() == NACK) {
            std::cout << "IMU configuration failed: " + config.describeI2CDevice((DynamicConfig::I2CDeviceConfig)imuConfig) << std::endl;
            return false;
        }
        else {
            firmwareData->clearRequest();
        }
    }

    // Configures servo drivers
    for (uint8_t i = 0; i < config.numServoDrivers(); i++) {
        DynamicConfig::I2CDeviceConfig i2cConfig = config.getServoDriver(i);
        Request request = firmwareData->writeRequest(CONFIG_SERVO_DRIVER + i2cConfig.onBusChain);

        // I2C bus or BusChain id
        firmwareData->writeByte(i2cConfig.busId);
        // BusChain channel
        if (i2cConfig.onBusChain) {
            firmwareData->writeByte(i2cConfig.channel);
        }
        firmwareData->sendAll();
        request->WaitAsync().get();
        if (request->IsTimedOut() || request->GetResponseHeader() == NACK) {
            std::cout << "Servo driver configuration failed: " + config.describeI2CDevice(i2cConfig) << std::endl;
            return false;
        }
        else {
            firmwareData->clearRequest();
        }
    }

    // Configures servos
    for (uint8_t i = 0; i < config.numServos(); i++) {
        DynamicConfig::ServoConfig servoConfig = config.getServo(i);
        Request request = firmwareData->writeRequest(CONFIG_SERVO);

        // Servo driver id
        firmwareData->writeByte(servoConfig.servoDriverId);
        // Servo channel
        firmwareData->writeByte(servoConfig.channel);
        firmwareData->sendAll();
        request->WaitAsync().get();
        if (request->IsTimedOut() || request->GetResponseHeader() == NACK) {
            std::cout << "Servo configuration failed: " + config.describeServo(servoConfig) << std::endl;
            return false;
        }
        else {
            firmwareData->clearRequest();
        }
    }

    // Configures foc motors
    for (uint8_t i = 0; i < config.numFOCMotors(); i++) {
        DynamicConfig::FOCMotorConfig focMotorConfig = config.getFOCMotor(i);
        Request request = firmwareData->writeRequest(CONFIG_FOC_MOTOR);
        //Motor port
        firmwareData->writeByte(focMotorConfig.port);
        firmwareData->sendAll();
        request->WaitAsync().get();
        if (request->IsTimedOut() || request->GetResponseHeader() == NACK) {
            std::cout << "FOC Motor configuration failed: " + config.describeFOCMotor(focMotorConfig) << std::endl;
            return false;
        }
        else {
            firmwareData->clearRequest();
        }
    }

    // Tells hardware to exit config mode
    Request request = firmwareData->writeRequest(CONFIG_END);
    firmwareData->sendAll();
    request->WaitAsync().get();
    if (request->IsTimedOut() || request->GetResponseHeader() == NACK) {
        std::cout << "Configuration completion denied" << std::endl;
        return false;
    }
    else {
        firmwareData->clearRequest();
    }

    // Finishes configuration if everything was succesful
    configFlag = true;
    return true;
}

void calibration() {
    std::cout << "Calibrating IMU" << std::endl;
    // Calibrates IMUs
    for (int i = 0; i < 1; i++) {
        imus[i].calibrate();
        // Waits for IMU to be calibrated
        while (!imus[i].isCalibrated()) {
            sleep(100);
        }
        imus[i].reset();
    }
    
    // Implement: Calibrates actuators?

    calibrationFlag = true;
}

void homing() {
    //Runs automatic homing procedure
    // Allows motors to tighten on thimble
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        motors[i].setForceTarget(0);
    }
    sleep(homingTime[0]);

    // Homes each motor for a certain amount of time
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        printf("Homing motor %d\n", i);
        motors[i].beginHoming();
        motors[i].setForceTarget(-homingPower);
        sendMotorCommands();
        sleep(homingTime[1]);
        motors[i].endHoming();
        motors[i].setForceTarget(0);
        sendMotorCommands();
    }
    homeFlag = true;
}

void firmwareReadHandler(std::shared_ptr<MinBiTCore> protocol, Request request) {
    // Ensures request did not time out
    if (request->IsTimedOut())
    {
        if (request->GetHeader() == PING) {
            std::cout << "Connection request timed out. Trying again" << std::endl;
            // Sends another ping
            protocol->writeRequest(PING);
            protocol->sendAll();
        }
        return;
    }
    // Gets response header
    uint8_t response = request->GetResponseHeader();
    //Reads serial packets
    switch (request->GetHeader()) {
        case PING: {
            if (response == ACK) {
                // If PING acknowledged
                aliveFlag = true;
                std::cout << "Handshake complete" << std::endl;
                //Sends initial sensor data request
                firmwareData->writeRequest(SENSOR_DATA);
                firmwareData->sendAll();
            }
            else {
                // If PING not acknowledged
                std::cout << "Connection denied" << std::endl;
                // Sends another ping
                protocol->writeRequest(PING);
                protocol->sendAll();
            }
            break;
        }
        case SENSOR_DATA: {
            if (response == ACK) {
                // Confirm response length matches expected
                if (request->GetPayloadLength() != config.getSensorDataLength()) {
                    std::cout << "Sensor data length incorrect" << std::endl;
                    protocol->flush();
                    break;
                }

                //Sends another sensor data request
                firmwareData->writeRequest(SENSOR_DATA);
                firmwareData->sendAll();

                //Processes magnetic encoder data
                for (uint8_t i = 0; i < NUM_MOTORS; i++) {
                    //Reads data
                    uint16_t sensorData = protocol->readData<uint16_t>();
                    //Updates sensor data
                    magEncoders[i].updateData(sensorData);
                }

                //Processes magnetic tracker data
                for (uint8_t i = 0; i < 2; i++) {
                    //Reads sensor data
                    std::array<int16_t, 3> sensorData;

                    //Reads in sensor data
                    for (uint8_t i = 0; i < 3; i++) {
                        sensorData[i] = protocol->readData<int16_t>();
                    }

                    magTrackers[i].storeRawData(sensorData);
                }

                // Processes imu data
                for (uint8_t i = 0; i < 1; i++) {
                    //Reads sensor data
                    int16_t x, y, z;
                    x = protocol->readData<int16_t>();
                    y = protocol->readData<int16_t>();
                    z = protocol->readData<int16_t>();
                    imus[i].updateAccelData(x, y, z);

                    x = protocol->readData<int16_t>();
                    y = protocol->readData<int16_t>();
                    z = protocol->readData<int16_t>();
                    imus[i].updateGyroData(x, y, z);
                }

                //Runs processing
                processCondition.notify_one();
            }
            else {
                std::cout << "Sensor data request denied" << std::endl;
            }
            break;
        }
        default: {
            // Handle actuator non-acknowledge errors
            if (response == NACK) {
                switch (request->GetHeader()) {
                    case SERVO_SIGNAL:
                    case FOC_VELOCITY:
                    case FOC_TORQUE:
                        std::cout << "Actuator command denied" << std::endl;
                        break;
                }
            }

            break;
        }
    }
}

void appReadHandler(std::shared_ptr<MinBiTCore> protocol, Request request) {
    // Ensures request did not time out
    if (request->IsTimedOut())
    {
        return;
    }
    switch (request->GetHeader()) { // Use DataProtocol's `getHeader` method
        case SEND_NODE_DATA: {
            if (aliveFlag && homeFlag) {
                // Sends node data to application
                appData->writeRequest(ACK);

                // Writes thimble position
                appData->writeVector3d(motorPlex.getPosition());

                // Writes thimble orientation
                appData->writeQuaterniond(thimble.getTrueOrient());

                // Writes packet
                appData->sendAll();
            }
            else {
				// Writes error response if not alive or not homed
                protocol->writeByte(NACK);
                protocol->sendAll();

                if (!homeFlag) {
                    std::cout << "Node data request received before homing" << std::endl;
                }
            }
        
            break;
        }
        case FORCE_FEEDBACK: {
            if (homeFlag) {
                // Writes feedback acknowledgement
                protocol->writeByte(ACK);
                protocol->sendAll();

                // Handle force feedback request
                // Reads feedback force in x, y, z format
                Vector3d feedbackForce = protocol->readVector3d();

                // Sets force target
                motorPlex.setForceTarget(feedbackForce);
            }
            else {
                // Writes error response if not homed
                protocol->writeByte(NACK);
                protocol->sendAll();
				std::cout << "Force feedback request received before homing" << std::endl;
            }
            break;
        }
        case COLLISION_FEEDBACK: {
            if (homeFlag) {
                // Writes feedback acknowledgement
                protocol->writeByte(ACK);
                protocol->sendAll();

                // Handle node feedback request
                // Reads collision point and normal as well as time to collision
                Vector3d collisionPoint = protocol->readVector3d();
                Vector3d collisionNormal = protocol->readVector3d();
                double timeToCollision = protocol->readFloat();

                if (collisionNormal.norm() > 0) {
                    // Sets collision target target in DRIFTPlex
                    motorPlex.setCollisionTarget(collisionPoint, collisionNormal.normalized(), timeToCollision);
                }
                else {
                    motorPlex.disableCollisionControl();
                }
            }
            else {
                // Writes error response if not homed
                protocol->writeByte(NACK);
                protocol->sendAll();
                std::cout << "Collision feedback request received before homing" << std::endl;
            }
            break;
        }
    }
}

void kinematicSolver() {
    bool printing = false;
    if (printTimer.elapsedMillis() > 500) {
        printTimer.reset();
        printing = true;
    }
    // Processing time step
	double stepTime = processTimer.elapsedSeconds();
	processTimer.reset();

    // Updates plex data
    motorPlex.updateData(stepTime);
    
    if (homeFlag) {
        // Runs haptic simulation
		motorPlex.updateController(stepTime);
        if (printing) {
            Vector3d position = motorPlex.getPosition();
            std::cout << "Position:\n" << toString(position) << std::endl;
        }
    }
}

void sendMotorCommands() {
    if (aliveFlag && configFlag) {
        for (uint8_t i = 0; i < NUM_MOTORS; i++) {
            switch (motors[i].getMode()) {
            case HydraFOCMotor::FORCE:
                firmwareData->writeRequest(FOC_TORQUE);
                firmwareData->writeByte(i);
                firmwareData->writeFloat(motors[i].getTorqueTarget());
                break;
            case HydraFOCMotor::VELOCITY:
                firmwareData->writeRequest(FOC_VELOCITY);
                firmwareData->writeByte(i);
                firmwareData->writeFloat(motors[i].getVelocityTarget());
                break;
            }
            firmwareData->sendAll();
        }
    }
}

void sendServoCommands() {
    //Sends data to servos
    if (aliveFlag && configFlag) {
        for (uint8_t i = 0; i < config.numServos(); i++) {
            // Writes data header
            firmwareData->writeRequest(SERVO_SIGNAL);
            // Writes servo id
            firmwareData->writeByte(i);
            // Writes servo power
            firmwareData->writeInt16(servos[i].getSignal());
            firmwareData->sendAll();
        }
    }
}

void processingThread() {
    while (true) {
        // Waits for notification that sensor data is ready
        {
            std::unique_lock<std::mutex> lock(processMutex);
            processCondition.wait(lock, [] { return true; });
        }

        //Runs kinematic solver
        kinematicSolver();

        // Sends actuator commands
        sendMotorCommands();
    }
}


