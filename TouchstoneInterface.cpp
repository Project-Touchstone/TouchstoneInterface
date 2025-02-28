// TouchstoneInterface.cpp : Defines the entry point for the application.

#include "TouchstoneInterface.h"

using namespace std;
using namespace SerialHeaders;
using namespace Eigen;

// Encoder objects
MagEncoder magEncoders[NUM_MOTORS * 2];

DRIFTPlex motorPlex;
DRIFTMotor motors[NUM_MOTORS];

const uint16_t calibrationTime[2] = { 3000, 500 };
const uint16_t homingTime = 20000;

Vector2d homePoints[NUM_MOTORS];

//Finger cap radius
double capRadius = 18.822;

//Wall plane
Vector2d planePoint;
Vector2d planeNormal;

volatile bool calibrationFlag = true;
volatile bool homeFlag = false;

int main()
{
    //Initializes parameters
    uint8_t error = setup();
    if (error > 0) {
        return error;
    }

    //Creates main threads
    std::thread generalThread(generalScheduler);
    std::thread serialThread(serialInterface);

    return 0;
}

// The setup function runs once when you press reset or power on the board.
uint8_t setup() {
    // Initialize serial communication at 115200 bits per second:
    uint8_t errorOpening = SerialInterface::begin(SERIAL_PORT, 115200);

    // If connection fails, return the error code otherwise, display a success message
    if (errorOpening != 1) return errorOpening;
    printf("Successful connection to %s\n", SERIAL_PORT);


    //Initializes DRIFT motor outlet points (x, y)
    Vector2d a1, a2, a3;
    homePoints[0] << 87.21284, 36.20728;
    a1 << -cos(EIGEN_PI / 6) * capRadius, -sin(EIGEN_PI / 6) * capRadius;
    homePoints[0] += a1;
    homePoints[1] << -12.25, -93.63217;
    a2 << 0, capRadius;
    homePoints[1] += a2;
    homePoints[2] << -74.96284, 57.4249;
    a3 << cos(EIGEN_PI / 6) * capRadius, -sin(EIGEN_PI / 6) * capRadius;
    homePoints[2] += a3;

    //Initializes wall plane
    planePoint << 0, 0;
    planeNormal << -1, 0;

    //Attaches encoders to motors
    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].attach(&magEncoders[i / 2], &magEncoders[i / 2 + 1]);
    }
    //Gives homing points and motors to DRIFTPlex
    motorPlex.attach(motors, homePoints, 3);

    return 0;
}

/*--------------------------------------------------*/
/*---------------------- Threads ---------------------*/
/*--------------------------------------------------*/
void sleep(uint32_t ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}
void generalScheduler() {
    encoderCalibration();
    positionHoming();
}

void encoderCalibration() {
    calibrationFlag = true;

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

    calibrationFlag = false;
}

void positionHoming() {
    //Sets motors to homing mode and waits
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        motors[i].beginHoming();
    }
    sleep(homingTime);
    //Turns off homing mode
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        motors[i].endHoming();
    }

    homeFlag = true;
}

void serialInterface() {
    while (true) {
        //Wait until serial data is available
        if (SerialInterface::processPacket()) {
            switch (SerialInterface::getHeader()) {
            case PING_ACK:
                //Do something to acknowledge ping
                SerialInterface::clearPacket();
                break;
            case SENSOR_DATA:
                //Processes sensor data
                if (SerialInterface::available() >= 5) {
                    //Reads sensor ID and data
                    uint8_t sensorID = SerialInterface::readByte();
                    int16_t sensorData[2];
                    sensorData[0] = SerialInterface::readData<int16_t>();
                    sensorData[1] = SerialInterface::readData<int16_t>();
                    //Ensures floating point numbers are legitimate values
                    bool isValid = true;
                    for (uint8_t i = 0; i < 2; i++) {
                        isValid &= (!isnan(sensorData[i]) && !isinf(sensorData[i]));
                    }
                    //Ensures sensor id is within range
                    if (sensorID < sizeof(magEncoders) / sizeof(magEncoders[0]) && isValid) {
                        magEncoders[sensorID].updateData(sensorData);
                    }
                    //Runs kinematic solver
                    kinematicSolver();
                }
                else if (SerialInterface::isEnded()) {
                    SerialInterface::clearPacket();
                }
                break;
            case PWM_CYCLE:
                // Sends servo powers

                // Sends data header
                SerialInterface::sendByte(SERVO_POWER);
                for (uint8_t i = 0; i < NUM_MOTORS; i++) {
                    // Sends motor id
                    SerialInterface::sendByte(i);
                    // Sends motor power
                    SerialInterface::sendFloat32((float)motors[i].getPower());
                }
                // Sends end of data frame
                SerialInterface::sendEnd();
                break;
            }
        }
    }
}

void kinematicSolver() {
    //Updates localization
    if (homeFlag) {
        updateSim();
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

std::string toString(const Eigen::VectorXd mat) {
    std::stringstream ss;
    ss << mat;
    return ss.str().c_str();
}

void updateSim() {
    motorPlex.localize();
    Vector2d loc = motorPlex.getPosition();
    //Serial.println(toString(loc));
    //Serial.println();

    double distToPlane = (loc - planePoint).dot(planeNormal);
    
    Vector2d n = distToPlane * planeNormal;
    if (distToPlane <= 0) {
        //If inside wall
        //Targets closest point on wall
        Vector2d closestPoint = loc - n;
        //Sets POSITION target
        motorPlex.setPositionLimit(closestPoint, true);
    }
    else {
        //If outside wall
        Vector2d vhat = motorPlex.getVelocity().normalized();
        Vector2d slant = -pow(n.norm(), 2) / vhat.dot(n) * vhat;
        motorPlex.setPositionLimit(loc + slant, false);
    }
    motorPlex.updateController();
}


