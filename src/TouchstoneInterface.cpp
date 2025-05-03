// TouchstoneInterface.cpp : Defines the entry point for the application.

#include "TouchstoneInterface.h"

using namespace SerialHeaders;
using namespace Eigen;
using namespace boost;
using namespace Utils;

//Serial interface object
SerialInterface serial;

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

//Homing power
const double homingPower = 1;
//Homing time
const uint16_t homingTime[2] = { 10000, 5000 };

//Finger cap parameters
double capRadius = 18.822;
double capHeight = 30.25;

//Servo power multiplier
float servoPowerMultiplier = 32767;

//Wall plane
Vector3d planePoint;
Vector3d planeNormal;

//Processing queue
std::queue<int> processingQueue;
std::mutex queueMutex;
std::condition_variable queueCondition;

bool calibrationFlag = false;
bool homeFlag = false;
bool aliveFlag = false;
bool processingDone = false;

high_resolution_clock::time_point lastPrintTime;

int main()
{
    //Initializes parameters
    uint8_t error = setup();
    if (error > 0) {
        return error;
    }

    serial.flushUntilTimeout();
    cout << "Flush complete" << endl;

    thread generalThread(generalScheduler);
    thread serialThread(serialInterface);
    thread processingThread(processing);

    generalThread.join();
    serialThread.join();
    processingThread.join();
    return 0;
}

// The setup function runs once when you press reset or power on the board.
uint8_t setup() {
    // Initialize serial communication at 115200 bits per second:
    // If connection fails, return the error code otherwise, display a success message
    if (!serial.begin(SERIAL_PORT, BAUD_RATE, TIMEOUT)) return 1;
    printf("Successful connection to %s\n", SERIAL_PORT);

    //Initializes DRIFT motor outlet points (x, y, z)
    homePoints[0] = { 0, 0, -124.404 };
    homePoints[1] = { -136.127, -61.985, 124.404 };
    homePoints[2] = { 12.252, 152.579, 124.404 };
    homePoints[3] = { 123.881, -83.203, 124.404 };

    offsets[0] = { 0, capHeight / 2, capRadius };
    offsets[1] = { (double)(capRadius * cos(EIGEN_PI / 6)), capHeight / 2, (double) (-capRadius * sin(EIGEN_PI / 6))};
    offsets[2] = { 0, -capHeight / 2, 0 };
    offsets[3] = { (double)(- capRadius * cos(EIGEN_PI / 6)), capHeight / 2, (double)(-capRadius * sin(EIGEN_PI / 6))};

    //Initializes wall plane
    planePoint = Vector3d::Zero();
    planeNormal << -1, 0, 0;

    //Attaches encoders to motors
    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].attach(&magEncoders[i * 2], &magEncoders[i * 2 + 1]);
    }
    //Gives homing points and motors to DRIFTPlex
    motorPlex.attach(motors, homePoints, offsets);

	//Attaches magnetic trackers to thimble object
	thimble.attachMagTrackers(magTrackers);

    //Sets tracker orientations
	magTrackers[0].setSensorOrientation(eulerToQuat(Vector3d(EIGEN_PI, 0, 0)));
	magTrackers[1].setSensorOrientation(eulerToQuat(Vector3d(EIGEN_PI, EIGEN_PI, 0)));

    //Sets imu ranges
    imu.setRanges(IMU::ACCELRANGE_2G, IMU::GYRORANGE_250DPS);
	// Sets IMU orientation offset
	imu.setOrientationOffset(eulerToQuat(Vector3d(-EIGEN_PI/2, 0, EIGEN_PI/2)));
    
    return 0;
}

/*--------------------------------------------------*/
/*---------------------- Threads ---------------------*/
/*--------------------------------------------------*/
void generalScheduler() {
    while (!aliveFlag) {
        sleep(10);
    }
    cout << "Calibrating encoders" << endl;
    encoderCalibration();
    cout << "Homing positions" << endl;
    positionHoming();
    cout << "Homing complete" << endl;
}

void encoderCalibration() {
    cout << "Calibrating IMU" << endl;
    // Calibrates IMU
    imu.calibrate();
    // Waits for IMU to be calibrated
    while (!imu.isCalibrated()) {
        sleep(100);
    }
    imu.reset();
    cout << "IMU calibrated" << endl;

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
}

void positionHoming() {
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

void serialInterface() {
    uint16_t count = 0;
    while (true) {
        //Update serial data
        serial.update(TIMEOUT);
        //Wait until header is ready
        if (serial.headerReady()) {
            switch (serial.getHeader()) {
                case PING_ACK:
                    aliveFlag = true;
                    cout << "Handshake complete" << endl;
                    serial.clearPacket();
                    break;
                case MAGENCODER_DATA:
                    //Processes sensor data
                    if (serial.available() >= 5) {
                        //Reads sensor ID and data
                        uint8_t sensorID = serial.readByte();
                        std::array<int16_t, 2> sensorData;
                        
                        // Reads in sensor data
                        for (uint8_t i = 0; i < 2; i++) {
                            sensorData[i] = serial.readData<int16_t>();
                        }
                        //Ensures sensor id is within range
                        if (sensorID < NUM_MOTORS*2) {
                            magEncoders[sensorID].storeRawData(sensorData);
                            queueMutex.lock();
                            processingQueue.push(sensorID);
                            queueMutex.unlock();
                            queueCondition.notify_one();
                        }
                        
                        // Clears packet
                        serial.clearPacket();
                    }
                    break;
                case MAGTRACKER_DATA:
                    //Processes sensor data
                    if (serial.available() >= 7) {
                        //Reads sensor ID and data
                        uint8_t sensorID = serial.readByte();
                        std::array<int16_t, 3> sensorData;

                        //Reads in sensor data
                        for (uint8_t i = 0; i < 3; i++) {
                            sensorData[i] = serial.readData<int16_t>();
                        }
                        //Ensures sensor id is within range
                        if (sensorID < 2) {
                            magTrackers[sensorID].storeRawData(sensorData);
                        }

                        // Clears packet
                        serial.clearPacket();
                    }
                    break;
                case IMU_DATA:
                    if (serial.available() >= 13) {
                        //Reads sensor ID and data
                        uint8_t sensorID = serial.readByte();

                        int16_t x, y, z;
                        x = serial.readData<int16_t>();
                        y = serial.readData<int16_t>();
                        z = serial.readData<int16_t>();
                        imu.updateAccelData(x, y, z);

                        x = serial.readData<int16_t>();
                        y = serial.readData<int16_t>();
                        z = serial.readData<int16_t>();
                        imu.updateGyroData(x, y, z);

                        //Clears packet
                        serial.clearPacket();
                    }
                    break;
                case PWM_CYCLE:
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
                    serial.clearPacket();
                    //printf("Sensor Read Count: %d\n", count);
                    count = 0;
                    break;
                default:
                    //printf("Invalid header: %d\n", serial.getHeader());
                    serial.clearPacket();
                    break;
            }
        }
        else if (processingDone) {
            processingDone = false;
            for (uint8_t i = 0; i < NUM_MOTORS; i++) {
                // Sends data header
                serial.sendByte(SERVO_POWER);
                // Sends motor id
                serial.sendByte(i);
                // Sends motor power
                serial.sendInt16(static_cast<int16_t>(motors[i].getPower() * servoPowerMultiplier));
            }
        }
        else if (serial.timedout()) {
            //If serial read times out
            aliveFlag = false;
            cout << "Waiting for signal..." << endl;
            serial.sendByte(PING);
            serial.clearPacket();
        }
    }
}

void kinematicSolver() {
    bool printing = false;
    if (high_resolution_clock::now() - lastPrintTime > milliseconds(500)) {
        lastPrintTime = high_resolution_clock::now();
        printing = true;
    }
    //Updates localization
    imu.updateOrientation();
    if (printing) {
        Quaterniond orientation = imu.getOrientation();
		Vector3d euler = quatToEuler(orientation);
        cout << "Orientation:\n" << toString(euler*180/EIGEN_PI) << endl;
    }
    if (homeFlag) {
		//Updates home point offsets based on IMU orientation
        motorPlex.updateOrientation(imu.getOrientation());
        // Runs localization algorithm
        motorPlex.localize();
        // Performs yaw estimation using gyro prediction
        double yawEstimate = motorPlex.estimateRotationChange(Quaterniond(0, 0, 0, 1), imu.getPredictedYawChange());
        // Passes yaw data back though Kalman filter to update orientation
        imu.updateYaw(yawEstimate);
        // Updates offsets again
		motorPlex.updateOrientation(imu.getOrientation());
        // Updates thimble data
        thimble.update();
        // Updates motor plex external position offset
		motorPlex.updatePositionOffset(thimble.getInnerCapPos());
        // Finds true orientation
		Quaterniond trueOrient = imu.getOrientation() * thimble.getInnerCapOrient();
        // Runs haptic simulation
        updateSim(trueOrient);
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

void updateSim(Quaterniond trueOrient) {
    Vector3d loc = motorPlex.getPosition();

    double distToPlane = (loc - planePoint).dot(planeNormal);
    
    Vector3d n = distToPlane * planeNormal;
    if (distToPlane <= 0) {
        //If inside wall
        //Targets closest point on wall
        Vector3d closestPoint = loc - n;
        //Sets POSITION target
        motorPlex.setPositionLimit(closestPoint, true);
    }
    else {
        //If outside wall
        Vector3d vhat = motorPlex.getVelocity().normalized();
        Vector3d slant = -distToPlane / vhat.dot(planeNormal) * vhat;
        motorPlex.setPositionLimit(loc + slant, false);
    }
    motorPlex.updateController();
}

void processing() {
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
    }
}


