#include <DRIFTMotor.h>
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
#define servoDriverPort 8

#define NUM_MOTORS 4

using namespace Eigen;

const uint8_t servoChannels[NUM_MOTORS] = {0, 1, 2, 3};
const uint8_t encoderPorts[NUM_MOTORS][2] = {{6, 7}, {10, 5}, {11, 3}, {0, 1}};

DRIFTMotor motors[NUM_MOTORS];


const TickType_t calibrationTime[2] = {pdMS_TO_TICKS(3000), pdMS_TO_TICKS(500)};
const TickType_t homingTime = pdMS_TO_TICKS(20000);

//Positions of DRIFT motor outlets
//Vector3f h1, h2, h3;

//Finger cap radius
//float capRadius = 18.822;

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
      //localize();
    }
    //Updates model predictive control
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
      motors[i].updateMPC();
      ServoController::updatePWMCompute(servoChannels[i]);
      num_t motorNum = i;
      xQueueSend(servoQueue, &motorNum, 0);
    }
  }
}

String toString(const Eigen::VectorXf &mat){
    std::stringstream ss;
    ss << mat;
    return ss.str().c_str();
}

/*void localize() {
  Vector3f v1, v2, Xn, Yn, Zn, s;
  float r1, r2, r3, i, d, j, x, y, z, z2;

  r1 = motors[0].getPosition();
  r2 = motors[1].getPosition();
  r3 = motors[2].getPosition();

  v1 = h2-h1;
  v2 = h3-h1;

  Xn = v1.normalized();
  Zn = v1.cross(v2).normalized();
  Yn = Xn.cross(Zn);

  i = Xn.dot(v2);
  d = Xn.dot(v1);
  j = Yn.dot(v2);

  x = (pow(r1, 2)-pow(r2, 2)+pow(d, 2))/(2*d);
  y = (pow(r1, 2)-pow(r3, 2)+pow(i, 2)+pow(j, 2))/(2*j) - i/j*x;
  z = sqrt(max(0., pow(r1, 2)-pow(x, 2)-pow(y,2)));

  s = h1 + x*Xn + y*Yn + z*Zn;

  Serial.println(toString(s));
  Serial.println();
}*/

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

  //Initializes DRIFT motor outlet points (x, y, z)
  /*Vector3f a1, a2, a3;
  h1 << 87.21284, 36.20728, 0;
  a1 << -cos(PI/6)*capRadius, -sin(PI/6)*capRadius, 0;
  h1 += a1;
  h2 << -12.25, -93.63217, 0;
  a2 << 0, capRadius, 0;
  h2 += a2;
  h3 << -74.96284, 57.4249, 0;
  a3 << cos(PI/6)*capRadius, -sin(PI/6)*capRadius, 0;
  h3 += a3;*/
  
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
