#include <DRIFTMotor.h>
#include <DRIFTPlex.h>
#include <ServoController.h>
#include <BusChain.h>
#include <math.h>
#include <ArduinoEigenDense.h>

#define I2C_CORE 0
#define APP_CORE 1

#define SER 4
#define CLK 15
#define RCLK 2

#define interruptPin 5
#define servoDriverPort 11

#define NUM_MOTORS 3

using namespace Eigen;

const uint8_t servoChannels[3] = {0, 1, 14};
const uint8_t encoderPorts[3][2] = {{13, 15}, {7, 6}, {8, 9}};

DRIFTPlex motorPlex;
DRIFTMotor motors[3];

const TickType_t calibrationTime[2] = {pdMS_TO_TICKS(3000), pdMS_TO_TICKS(500)};
const TickType_t homingTime = pdMS_TO_TICKS(20000);

Vector2f homePoints[3];

//Finger cap radius
float capRadius = 18.822;

//Wall plane
Vector2f planePoint;
Vector2f planeNormal;

// Define task handles
TaskHandle_t generalSchedulerHandle;
TaskHandle_t pwmSchedulerHandle;
TaskHandle_t sensorReadHandle;
TaskHandle_t servoControllerHandle;
TaskHandle_t encoderCalibrationHandle;
TaskHandle_t positionHomingHandle;
TaskHandle_t kinematicSolverHandle;
TaskHandle_t encoderInterpolationHandle;

volatile bool calibrationFlag = true;
volatile bool homeFlag = false;

uint64_t startTime;

typedef uint8_t num_t;
QueueHandle_t interpolationQueue;
QueueHandle_t servoQueue;

void IRAM_ATTR onPWMStart() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;  
  ServoController::updatePWMTime();
  vTaskNotifyGiveFromISR(pwmSchedulerHandle, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );  
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/
void TaskGeneralScheduler(void *pvParameters) {
  for (;;) {
    xTaskNotifyGive(encoderCalibrationHandle);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    xTaskNotifyGive(positionHomingHandle);

    //Deletes current task
    vTaskDelete(NULL);
  }
}

void TaskPWMScheduler(void *pvParameters) {
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (!calibrationFlag) {
      xTaskNotifyGive(kinematicSolverHandle);
    } else {
      for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        ServoController::updatePWMCompute(servoChannels[i]);
        num_t motorNum = i;
        xQueueSend(servoQueue, &motorNum, 0);
      }
    }
  }
}

void TaskSensorRead(void *pvParameters) {
  for (;;) {
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
      for (uint8_t j = 0; j < 2; j++) {
        motors[i].updateSensor(j);
        //Writes current sensor to queue for interpolation
        num_t sensorNum = i*2 + j;
        xQueueSend(interpolationQueue, &sensorNum, 0);
      }
    }
  }
}

void TaskServoController(void *pvParameters) {
  (void)pvParameters;
  for (;;) {
    num_t motorNum;
    xQueueReceive(servoQueue, &motorNum, portMAX_DELAY);
    ServoController::updatePWMDriver(servoChannels[motorNum]);
  }
}

void TaskEncoderCalibration(void *pvParameters) {
  (void)pvParameters;

  for (;;) {
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
    //Resets PWM timing on servo controller
    ServoController::reset();

    calibrationFlag = false;

    //Gives control back to general scheduler
    xTaskNotifyGive(generalSchedulerHandle);

    //Deletes current task
    vTaskDelete(NULL);
  }
}

void TaskPositionHoming(void *pvParameters) {
  (void)pvParameters;

  for (;;) {
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
}

void TaskKinematicSolver(void *pvParameters) {
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
      } else {
        motors[i].updateMPC();
      }
      ServoController::updatePWMCompute(servoChannels[i]);
      num_t motorNum = i;
      xQueueSend(servoQueue, &motorNum, 0);
    }
  }
}

String toString(const Eigen::VectorXf mat){
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
  Vector2f n = distToPlane*planeNormal;
  if (distToPlane <= 0) {
    //If inside wall
    //Targets closest point on wall
    Vector2f closestPoint = loc - n;
    //Sets POSITION target
    motorPlex.setPositionLimit(closestPoint, true);
  } else {
    //If outside wall
    Vector2f vhat = motorPlex.getVelocity().normalized();
    Vector2f slant = -pow(n.norm(), 2)/vhat.dot(n)*vhat;
    motorPlex.setPositionLimit(loc + slant, false);
  }
  motorPlex.updateController();
}

void TaskEncoderInterpolation(void *pvParameters) {
  (void)pvParameters;

  for (;;) {
    //Recieves current motor from queue, blocks if not available
    num_t sensorNum;
    xQueueReceive(interpolationQueue, &sensorNum, portMAX_DELAY);
    motors[sensorNum/2].updateEncoder(sensorNum % 2);
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
  a1 << -cos(PI/6)*capRadius, -sin(PI/6)*capRadius;
  homePoints[0] += a1;
  homePoints[1] << -12.25, -93.63217;
  a2 << 0, capRadius;
  homePoints[1] += a2;
  homePoints[2] << -74.96284, 57.4249;
  a3 << cos(PI/6)*capRadius, -sin(PI/6)*capRadius;
  homePoints[2] += a3;

  //Gives homing points and motors to DRIFTPlex
  motorPlex.attach(motors, homePoints, 3);

  //Initializes wall plane
  planePoint << 0, 0;
  planeNormal << -1, 0;
  
  //Initializes buschain board
  BusChain::begin(SER, CLK, RCLK, 2);

  //Initializes servo driver board, flags connection error
  if (!ServoController::begin(servoDriverPort, interruptPin)) {
    Serial.println("Error connecting to servo driver");
    while (true) {
      
    }
  }

  //Attaches DRIFT motors to servo and encoder ports, flags encoder connection erros
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    int16_t err = motors[i].attach(servoChannels[i], encoderPorts[i][0], encoderPorts[i][1]);
    if (err > -1) {
      Serial.print("Error connecting to encoder port ");
      Serial.println(err);
      while (true) {

      }
    }
  }

  interpolationQueue = xQueueCreate(NUM_MOTORS*2, sizeof(num_t));
  servoQueue = xQueueCreate(NUM_MOTORS, sizeof(num_t));

  xTaskCreatePinnedToCore(TaskServoController, "Servo Controller", 2048, NULL, 5, &servoControllerHandle, I2C_CORE);

  xTaskCreatePinnedToCore(TaskEncoderCalibration, "Encoder Calibration", 2048, NULL, 1, &encoderCalibrationHandle, APP_CORE);

  xTaskCreatePinnedToCore(TaskPositionHoming, "Position Homing", 2048, NULL, 1, &positionHomingHandle, APP_CORE);

  xTaskCreatePinnedToCore(TaskKinematicSolver, "Kinematics Solver", 4096, NULL, 2, &kinematicSolverHandle, APP_CORE);

  xTaskCreatePinnedToCore(TaskEncoderInterpolation, "Encoder Interpolation", 2048, NULL, 3, &encoderInterpolationHandle, APP_CORE);

  xTaskCreatePinnedToCore(TaskSensorRead, "Sensor Read", 2048, NULL, 4, &sensorReadHandle, I2C_CORE);

  xTaskCreatePinnedToCore(TaskGeneralScheduler, "General Scheduler", 2048, NULL, 1, &generalSchedulerHandle, APP_CORE);
  
  xTaskCreatePinnedToCore(TaskPWMScheduler, "PWM Scheduler", 2048, NULL, 2, &pwmSchedulerHandle, APP_CORE);

  attachInterrupt(interruptPin, onPWMStart, RISING);
}

void loop() {

}
