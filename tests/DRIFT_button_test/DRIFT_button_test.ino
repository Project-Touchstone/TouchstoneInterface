#include <DRIFTMotor.h>
#include <ServoController.h>
#include <BusChain.h>
#include <math.h>

#define I2C_CORE 0
#define APP_CORE 1

#define SER 4
#define CLK 15
#define RCLK 2

#define servoChannel 14
#define interruptPin 5

#define servoDriverPort 11

const uint16_t encoderPorts[2] = {8, 9};

const float criticalPoints[3] = {32.72, 34.36, 36};
const float steepness = 8;

DRIFTMotor motor;

const TickType_t calibrationTime[2] = {pdMS_TO_TICKS(3000), pdMS_TO_TICKS(500)};
const TickType_t homingTime = pdMS_TO_TICKS(5000);

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
        ServoController::updatePWMCompute(servoChannel);
        xTaskNotifyGive(servoControllerHandle);
    }
  }
}

void TaskSensorRead(void *pvParameters) {
  for (;;) {
      for (uint8_t j = 0; j < 2; j++) {
        motor.updateSensor(j);
        //Writes current sensor to queue for interpolation
        num_t sensorNum = j;
        xQueueSend(interpolationQueue, &sensorNum, 0);
      }
  }
}

void TaskServoController(void *pvParameters) {
  (void)pvParameters;
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    ServoController::updatePWMDriver(servoChannel);
  }
}

void TaskEncoderCalibration(void *pvParameters) {
  (void)pvParameters;

  for (;;) {
    //Waits for scheduler notification
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    calibrationFlag = true;
    
    //Sets servo to low power for encoder amplitude and phase calibration
    motor.setPower(0.05);
    vTaskDelay(calibrationTime[0]);
    //Stops servo and delays to allow values to stabilize
    motor.setPower(0);
    
    vTaskDelay(calibrationTime[1]);
    //Resets all encoders
    motor.resetEncoders();
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
    motor.beginHoming();
    vTaskDelay(homingTime);
    //Turns off homing mode
    motor.endHoming();

    homeFlag = true;
    
    //Deletes current task
    vTaskDelete(NULL);
  }
}

void updateSim() {
  if (motor.getPosition() < criticalPoints[0]) {
      motor.setPositionLimit(criticalPoints[0]);
  } else if (motor.getPosition() < criticalPoints[1]) {
    motor.setForceTarget((motor.getPosition()-criticalPoints[0])*steepness);
  } else {
    motor.setPositionLimit(criticalPoints[2]);
  }
}

void TaskKinematicSolver(void *pvParameters) {
  for (;;) {
    //Waits for scheduler notification
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    Serial.print("Pos: ");
    Serial.print(motor.getPosition());
    Serial.print("\tVelocity: ");
    Serial.println(motor.getVelocity());
    //Updates simulation
    if (homeFlag) {
      updateSim();
    }
    //Updates model predictive control
    motor.updateMPC();
    ServoController::updatePWMCompute(servoChannel);
    xTaskNotifyGive(servoControllerHandle);
  }
}

void TaskEncoderInterpolation(void *pvParameters) {
  (void)pvParameters;

  for (;;) {
    //Recieves current motor from queue, blocks if not available
    num_t sensorNum;
    xQueueReceive(interpolationQueue, &sensorNum, portMAX_DELAY);
    motor.updateEncoder(sensorNum);
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {

  }
  
  BusChain::begin(SER, CLK, RCLK, 2);
  if (!ServoController::begin(servoDriverPort, interruptPin)) {
    Serial.println("Error connecting to servo driver");
    while (true) {
      
    }
  }
  int16_t err = motor.attach(servoChannel, encoderPorts[0], encoderPorts[1]);
  if (err > -1) {
    Serial.print("Error connecting to encoder port ");
    Serial.println(err);
    while (true) {

    }
  }
  
  interpolationQueue = xQueueCreate(2, sizeof(num_t));

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