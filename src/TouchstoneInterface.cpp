// TouchstoneInterface.cpp : Defines the entry point for the application.

#include "TouchstoneInterface.h"

using namespace SerialHeaders;
using namespace NetworkHeaders;
using namespace Eigen;
using namespace boost;
using namespace Utils;

//Firmware serial interface
MinBiTSerialClient firmware("Firmware Interface");

//Firmware protocol
std::shared_ptr<MinBiTCore> firmwareData;

//Application layer TCP/IP interface
MinBiTTcpServer application("Application Interface", SERVER_PORT);

//Application protocol
std::shared_ptr<MinBiTCore> appData;

// Encoder objects
MagEncoder magEncoders[NUM_MOTORS * 2];

//Magnetic tracker objects
MagTracker magTrackers[2];

//Thimble object
Thimble thimble;

//IMU object
IMU imu;

DRIFTPlex motorPlex;
DRIFTMotor motors[NUM_MOTORS];

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
double capRadius = 18.822;
double capHeight = 30.25;

//Servo power multiplier
float servoPowerMultiplier = 32767;

//Processing queue
std::queue<int> processingQueue;
std::mutex queueMutex;
std::condition_variable queueCondition;

std::mutex dataMutex;

// Whether connection with hardware is alive
bool aliveFlag = false;

// Whether hardware has been successfully configured
bool configFlag = false;

// Whether sensors and actuators have been calibrated
bool calibrationFlag = false;

// Whether device has been homed
bool homeFlag = false;

// Whether actuator commands are ready to send
bool processingDone = false;

//DEBUG: Time between processing cycles
Timer processTimer;

//DEBUG: Time between serial cycles
Timer printTimer;


uint8_t setup() {
    //Initializes DRIFT motor outlet points (x, y, z)
    homePoints[0] = { 0, 0, -124.404 };
    homePoints[1] = { -136.127, -61.985, 124.404 };
    homePoints[2] = { 12.252, 152.579, 124.404 };
    homePoints[3] = { 123.881, -83.203, 124.404 };

    offsets[0] = { 0, capHeight / 2, capRadius };
    offsets[1] = { (double)(capRadius * cos(EIGEN_PI / 6)), capHeight / 2, (double)(-capRadius * sin(EIGEN_PI / 6)) };
    offsets[2] = { 0, -capHeight / 2, 0 };
    offsets[3] = { (double)(-capRadius * cos(EIGEN_PI / 6)), capHeight / 2, (double)(-capRadius * sin(EIGEN_PI / 6)) };

    //Attaches encoders to motors
    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].attach(&magEncoders[i * 2], &magEncoders[i * 2 + 1]);
    }
    //Gives homing points and motors to DRIFTPlex
    motorPlex.attach(motors, homePoints, offsets);

    //Attaches magnetic trackers to thimble object
    thimble.attachMagTrackers(magTrackers);

    //Sets tracker orientations
    magTrackers[0].setSensorOrientation(eulerToQuat(Vector3d(-EIGEN_PI / 2, -EIGEN_PI, 0)));
    magTrackers[1].setSensorOrientation(eulerToQuat(Vector3d(EIGEN_PI / 2, 0, 0)));

    //Sets tracker positions
    magTrackers[0].setInitialPosition(Vector3d(0, 0, 1));
    magTrackers[1].setInitialPosition(Vector3d(0, 0, 1));

    //Sets imu ranges
    imu.setRanges(IMU::ACCELRANGE_2G, IMU::GYRORANGE_250DPS);
    // Sets IMU orientation offset
    imu.setOrientationOffset(eulerToQuat(Vector3d(-EIGEN_PI / 2, 0, EIGEN_PI / 2)));

    // Sets firmware data handler
    firmware.setReadHandler(&firmwareReadHandler);
    // Gets firmware data protocol
    firmwareData = firmware.getProtocol();
    // If connection fails, return the error code otherwise, display a success message
    if (!firmware.begin(SERIAL_PORT, BAUD_RATE)) return 1;
    printf("Successful connection to %s\n", SERIAL_PORT);

    //Creates application request handler
    application.setReadHandler(&appReadHandler);
    //Starts application interface
    application.begin();

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
    while (!aliveFlag) {
        sleep(10);
    }
    cout << "Calibration phase" << endl;
    calibration();
    cout << "Homing phase" << endl;
    homing();
    cout << "Homing complete" << endl;
}

void calibration() {
    cout << "Calibrating IMU" << endl;
    // Calibrates IMU
    imu.calibrate();
    // Waits for IMU to be calibrated
    while (!imu.isCalibrated()) {
        sleep(100);
    }
    imu.reset();
    cout << "Calibrating Encoders" << endl;
    //Sets servo to low power for encoder amplitude and phase calibration
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        motors[i].setPower(0.05);
    }
    sleep(calibrationTime[0]);
    //Stops servo and delays to allow values to stabilize
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        motors[i].setPower(0);
    }

    sleep(calibrationTime[1]);
    //Resets all encoders
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        motors[i].resetEncoders();
    }
    calibrationFlag = true;
    processTimer.reset();
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
        motors[i].setPower(-homingPower);
        sleep(homingTime[1]);
        motors[i].endHoming();
        motors[i].setForceTarget(0);
    }
    homeFlag = true;
}

void firmwareReadHandler(std::shared_ptr<MinBiTCore> protocol, std::shared_ptr<MinBiTCore::Request> request) {
    //Reads serial packets
    switch (request->GetHeader()) {
        case PING: {
            aliveFlag = true;
            cout << "Handshake complete" << endl;
            break;
        }
        case MAGENCODER_DATA: {
            //Processes sensor data
            if (data->getReadBufferSize() >= 5) {
                //Reads sensor ID and data
                uint8_t sensorID = data->readByte();
                std::array<int16_t, 2> sensorData;

                // Reads in sensor data
                for (uint8_t i = 0; i < 2; i++) {
                    sensorData[i] = data->readData<int16_t>();
                }
                //Ensures sensor id is within range
                if (sensorID < NUM_MOTORS * 2) {
                    magEncoders[sensorID].storeRawData(sensorData);
                    queueMutex.lock();
                    processingQueue.push(sensorID);
                    queueMutex.unlock();
                    queueCondition.notify_one();
                }

                // Clears packet
                data->clearReadPacket();
            }
            break;
        }
        case MAGTRACKER_DATA: {
            //Processes sensor data
            if (data->getReadBufferSize() >= 7) {
                //Reads sensor ID and data
                uint8_t sensorID = data->readByte();
                std::array<int16_t, 3> sensorData;

                //Reads in sensor data
                for (uint8_t i = 0; i < 3; i++) {
                    sensorData[i] = data->readData<int16_t>();
                }
                //Ensures sensor id is within range
                if (sensorID < 2) {
                    magTrackers[sensorID].storeRawData(sensorData);
                }

                // Clears packet
                data->clearReadPacket();
            }
            break;
        }
        case IMU_DATA: {
            if (data->getReadBufferSize() >= 13) {
                //Reads sensor ID and data
                uint8_t sensorID = data->readByte();

                int16_t x, y, z;
                x = data->readData<int16_t>();
                y = data->readData<int16_t>();
                z = data->readData<int16_t>();
                imu.updateAccelData(x, y, z);

                x = data->readData<int16_t>();
                y = data->readData<int16_t>();
                z = data->readData<int16_t>();
                imu.updateGyroData(x, y, z);

                //Clears packet
                data->clearReadPacket();
            }
            break;
        }
        case PWM_CYCLE: {
            //Runs kinematic solver (if calibrated)
            if (calibrationFlag) {
                queueMutex.lock();
                processingQueue.push(-1);
                queueCondition.notify_one();
                queueMutex.unlock();
            }
            else {
                //Otherwise jsut runs servos
                processingDone = true;
            }
            // Clears packet
            data->clearReadPacket();
            break;
        }
        default: {
            //printf("Invalid header: %d\n", serial.getHeader());
            data->clearReadPacket();
            break;
        }
    }
}

void appReadHandler(std::shared_ptr<MinBiTCore> protocol, std::shared_ptr<MinBiTCore::Request> request) {
    // Ensures request did not time out
    if (request->IsTimedOut())
    {
        return;
    }
    switch (request->GetHeader()) { // Use DataProtocol's `getHeader` method
        case SEND_NODE_DATA: {
            if (homeFlag) {
                // Writes node data response
                protocol->writeByte(ACK);

                // Writes thimble position
                protocol->writeVector3d(motorPlex.getPosition() / 1000.);

                // Writes thimble orientation
                protocol->writeQuaterniond(getTrueOrient());

                // Writes packet
                protocol->writePacket();
            }
            else {
				// Writes error response if not homed
                protocol->writeByte(NACK);
                protocol->writePacket();
            }
        
            break;
        }
        case FORCE_FEEDBACK: {
                if (homeFlag) {
                    // Writes feedback acknowledgement
                    protocol->writeByte(ACK);
                    protocol->writePacket();

                    // Handle force feedback request
                    // Reads feedback force in x, y, z format
                    Vector3d feedbackForce = protocol->readVector3d();

                    // Sets force target
                    motorPlex.setForceTarget(feedbackForce);
                }
                else {
                    // Writes error response if not homed
                    protocol->writeByte(NACK);
                    protocol->writePacket();
					cout << "Force feedback request received before homing" << endl;
                }
            break;
        }
        case COLLISION_FEEDBACK: {
                if (homeFlag) {
                    // Writes feedback acknowledgement
                    protocol->writeByte(ACK);
                    protocol->writePacket();

                    // Handle node feedback request
                    // Reads collision point and normal as well as time to collision
                    Vector3d collisionPoint = protocol->readVector3d() * 1000;
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
                    protocol->writePacket();
                }
            break;
        }
        default: {
            // Handle unknown request
            std::cerr << "Unknown request header: " << request->GetHeader() << std::endl;
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

    //Updates orientation
    imu.updateOrientation(stepTime);
    // Updates thimble data
    thimble.update(stepTime);
    Vector3d innerCapPos = thimble.getInnerCapPos();
    Quaterniond innerCapOrient = thimble.getInnerCapOrient();
    if (printing) {
        //Vector3d capEuler = quatToEuler(innerCapOrient);
        //Vector3d imuEuler = quatToEuler(imu.getOrientation());
        cout << "IMU Orientation:\n" << toString(imu.getOrientation().coeffs()) << endl;
        cout << "Cap Orientation:\n" << toString(innerCapOrient.coeffs()) << endl;
        //cout << "Position:\n" << toString(innerCapPos) << endl << endl;
    }
    if (homeFlag) {
		//Updates home point offsets based on IMU orientation
        motorPlex.updateOrientation(imu.getOrientation());
        // Updates motor plex external position offset
        motorPlex.updatePosOffset(innerCapPos);
        motorPlex.updateVelOffset(thimble.getInnerCapVel());
        // Runs localization algorithm
        motorPlex.localize(stepTime);
        if (printing) {
            Vector3d position = motorPlex.getPosition();
			cout << "Position:\n" << toString(position) << endl;
        }
        // Runs haptic simulation
		motorPlex.updateController();
    }
    //Updates model predictive control
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        if (homeFlag) {
            motors[i].updateMPC(motorPlex.getPredictedPos(i));
        }
        else {
            motors[i].updateMPC();
        }
    }
}

void processingThread() {
    while (true) {
        int task;
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            queueCondition.wait(lock, [] { return !processingQueue.empty(); });
            task = processingQueue.front();
            processingQueue.pop();
        }

        if (task == -1) {
            kinematicSolver();
            processingDone = true;
        }
        else {
            magEncoders[task].updateData();
        }

        //Writes serial packets
        if (aliveFlag && processingDone && !serialData->isReadPacketPending()) {
            processingDone = false;
            for (uint8_t i = 0; i < NUM_MOTORS; i++) {
                // Writes data header
                serialData->writeByte(SERVO_POWER);
                // Writes motor id
                serialData->writeByte(i);
                // Writes motor power
                serialData->writeInt16(static_cast<int16_t>(motors[i].getPower() * servoPowerMultiplier));
            }
        }
    }
}

Quaterniond getTrueOrient() {
    return imu.getOrientation()*thimble.getInnerCapOrient();
}

Vector3d getAngularVelocity() {
	return qRotate(imu.getOrientation(), imu.getGyroData() + thimble.getInnerCapAngVel());
}


