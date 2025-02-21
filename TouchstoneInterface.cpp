// TouchstoneInterface.cpp : Defines the entry point for the application.
//

#include "TouchstoneInterface.h"

using namespace std;
using namespace SerialHeaders;
using namespace Eigen;

int main()
{
    setup();
	return 0;
}

DRIFTPlex motorPlex;
DRIFTMotor motors[3];

const uint16_t calibrationTime[2] = { 3000, 500 };
const uint16_t homingTime = 20000;

Vector2f homePoints[3];

//Finger cap radius
float capRadius = 18.822;

//Wall plane
Vector2f planePoint;
Vector2f planeNormal;

volatile bool calibrationFlag = true;
volatile bool homeFlag = false;

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/
void TaskGeneralScheduler(void* pvParameters) {
    for (;;) {
        xTaskNotifyGive(encoderCalibrationHandle);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        xTaskNotifyGive(positionHomingHandle);

        //Deletes current task
        vTaskDelete(NULL);
    }
}

void TaskEncoderCalibration() {
    //Waits for scheduler notification
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    calibrationFlag = true;

    //Sets servo to low power for encoder amplitude and phase calibration
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        motors[i].setPower(0.05);
    }
    vTaskDelay(calibrationTime[0]);
    //Stops servo and delays to allow values to stabilize
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        motors[i].setPower(0);
    }

    vTaskDelay(calibrationTime[1]);
    //Resets all encoders
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        motors[i].resetEncoders();
    }

    calibrationFlag = false;

    //Gives control back to general scheduler
    xTaskNotifyGive(generalSchedulerHandle);

    //Deletes current task
    vTaskDelete(NULL);
}

void TaskPositionHoming() {
    //Waits for scheduler notification
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    //Sets motors to homing mode and waits
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        motors[i].beginHoming();
    }
    vTaskDelay(homingTime);
    //Turns off homing mode
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        motors[i].endHoming();
    }

    homeFlag = true;

    //Deletes current task
    vTaskDelete(NULL);
}

void TaskKinematicSolver(void* pvParameters) {
    for (;;) {
        //Waits for scheduler notification
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
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
            num_t motorNum = i;
            xQueueSend(servoQueue, &motorNum, 0);
        }
    }
}

std::string toString(const Eigen::VectorXf mat) {
    std::stringstream ss;
    ss << mat;
    return ss.str().c_str();
}

void updateSim() {
    motorPlex.localize();
    Vector2f loc = motorPlex.getPosition();
    //Serial.println(toString(loc));
    //Serial.println();

    float distToPlane = (loc - planePoint).dot(planeNormal);
    Serial.println(distToPlane);
    Vector2f n = distToPlane * planeNormal;
    if (distToPlane <= 0) {
        //If inside wall
        //Targets closest point on wall
        Vector2f closestPoint = loc - n;
        //Sets POSITION target
        motorPlex.setPositionLimit(closestPoint, true);
    }
    else {
        //If outside wall
        Vector2f vhat = motorPlex.getVelocity().normalized();
        Vector2f slant = -pow(n.norm(), 2) / vhat.dot(n) * vhat;
        motorPlex.setPositionLimit(loc + slant, false);
    }
    motorPlex.updateController();
}

void TaskEncoderInterpolation(void* pvParameters) {
    (void)pvParameters;

    for (;;) {
        //Recieves current motor from queue, blocks if not available
        num_t sensorNum;
        xQueueReceive(interpolationQueue, &sensorNum, portMAX_DELAY);
        motors[sensorNum / 2].updateEncoder(sensorNum % 2);
    }
}

// The setup function runs once when you press reset or power on the board.
void setup() {
    // Initialize serial communication at 115200 bits per second:
    Serial.begin(115200);
    while (!Serial) {

    }

    //Initializes DRIFT motor outlet points (x, y)
    Vector2f a1, a2, a3;
    homePoints[0] << 87.21284, 36.20728;
    a1 << -cos(PI / 6) * capRadius, -sin(PI / 6) * capRadius;
    homePoints[0] += a1;
    homePoints[1] << -12.25, -93.63217;
    a2 << 0, capRadius;
    homePoints[1] += a2;
    homePoints[2] << -74.96284, 57.4249;
    a3 << cos(PI / 6) * capRadius, -sin(PI / 6) * capRadius;
    homePoints[2] += a3;

    //Gives homing points and motors to DRIFTPlex
    motorPlex.attach(motors, homePoints, 3);

    //Initializes wall plane
    planePoint << 0, 0;
    planeNormal << -1, 0;
}

void TaskSerialInterface(void* pvParameters) {
    (void)pvParameters;
    for (;;) {
        // Waits for notification from scheduler or sensor reading
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (SerialInterface::processHeader()) {
            switch (SerialInterface::getHeader()) {
            case PING:
                // Sends ping acknowledgement
                SerialInterface::sendByte(PING_ACK);
                SerialInterface::clearHeader();
                break;
            case REQUEST_DATA:
                //Sends sensor data in queue
                //Sends data header
                SerialInterface::sendByte(SENSOR_DATA);
                while (uxQueueMessagesWaiting(sensorDataQueue) > 0) {
                    num_t sensorNum;
                    xQueueReceive(sensorDataQueue, &sensorNum, 0);
                    // Sends sensor id
                    SerialInterface::sendByte(sensorNum);
                    // Sends sensor data
                    SerialInterface::sendData<int16_t>(magSensors[sensorNum].rawX());
                    SerialInterface::sendData<int16_t>(magSensors[sensorNum].rawY());
                    SerialInterface::sendData<int16_t>(magSensors[sensorNum].rawZ());
                    // Sends end of data frame
                    SerialInterface::sendEnd();
                }
                SerialInterface::clearHeader();
                break;
            case SERVO_POWER:
                // Updates servo controller
                if (Serial.available() > 5 && !SerialInterface::isEnded()) {
                    uint8_t servoNum = SerialInterface::readByte();
                    float power = SerialInterface::readFloat();
                    //ServoController::setPower(servoNum, power);
                }
                else if (SerialInterface::isEnded()) {
                    SerialInterface::clearHeader();
                }
                break;
            }
        }
        else if (uxQueueMessagesWaiting(sensorDataQueue) > 0) {
            // Sends data ready header
            SerialInterface::sendByte(DATA_READY);
        }
    }
}


