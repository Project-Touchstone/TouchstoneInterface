//#include <Button.h>
#include <Wire.h>
#include <math.h>
#include "FastIMU.h"

#define SER 3
#define CLK 4
#define RCLK 5

#define targetPort 6
#define IMU_ADDRESS 0x68    //Change to the address of the IMU
#define PERFORM_CALIBRATION //Comment to disable startup calibration
MPU6050 IMU;               //Change to the name of any supported IMU! 

// Currently supported IMUS: MPU9255 MPU9250 MPU6886 MPU6500 MPU6050 ICM20689 ICM20690 BMI055 BMX055 BMI160 LSM6DS3 LSM6DSL QMI8658

calData calib = { 0 };  //Calibration data
AccelData accelData;    //Sensor data
GyroData gyroData;
MagData magData;

//Button trigger = Button(2);

uint32_t clockInterval = 1000;

bool state = false;

uint8_t serBuffer = 0;

uint8_t serRemaining = 0;

uint32_t timerStart = 0;

uint8_t addresses[2] = {0, 1};
bool enabled[2] = {true, true};

char strBuffer[32];

int selectPort(uint8_t port) {
  Wire.beginTransmission(0x70 + addresses[port/8]);
  Wire.write(1 << port%8);
  return Wire.endTransmission();
}

void setParams(uint8_t addrA, bool enA, uint8_t addrB, bool enB) {
  serBuffer = (addrB << 5) + (enB << 4) + (addrA << 1) + enA;
  serRemaining = 8;

  resetTimer();
}

void resetTimer() {
  timerStart = micros();
  state = false;
}

void disableOutput() {
  serBuffer = 0;
  serRemaining = 8;
  resetTimer();
}

void sendBit() {
  if (serRemaining > 0) {
    digitalWrite(SER, serBuffer & 1);
    serBuffer = serBuffer >> 1;
    digitalWrite(CLK, 1);
    if (serRemaining == 1) {
      digitalWrite(RCLK, 1);
    }
  }
}

void updateData() {
  uint32_t currTime = micros();
  if (currTime >= timerStart) {
    timerStart = currTime + clockInterval;
    state = !state;
    if (state) {
      sendBit();
    } else {
      if (serRemaining > 0) {
        serRemaining--;
      }
      digitalWrite(CLK, 0);
      digitalWrite(RCLK, 0);
      digitalWrite(SER, 0);
    }
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  pinMode(CLK, OUTPUT);
  pinMode(RCLK, OUTPUT);
  pinMode(SER, OUTPUT);

  disableOutput();

  while (serRemaining > 0) {
    updateData();
  }

  setParams(addresses[0], enabled[0], addresses[1], enabled[1]);

  while (serRemaining > 0) {
    updateData();
  }

  Wire.begin();
  Wire.setClock(400000);

  int err = selectPort(targetPort);
  if (err != 0) {
    Serial.print("Error selecting I2C port: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }

  err = IMU.init(calib, IMU_ADDRESS);
  if (err != 0) {
    Serial.print("Error initializing IMU: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }
  
  #ifdef PERFORM_CALIBRATION
    Serial.println("FastIMU calibration & data example");
    if (IMU.hasMagnetometer()) {
      delay(1000);
      Serial.println("Move IMU in figure 8 pattern until done.");
      delay(3000);
      IMU.calibrateMag(&calib);
      Serial.println("Magnetic calibration done!");
    }
    else {
      delay(5000);
    }

    delay(5000);
    Serial.println("Keep IMU level.");
    delay(5000);
    IMU.calibrateAccelGyro(&calib);
    Serial.println("Calibration done!");
    Serial.println("Accel biases X/Y/Z: ");
    Serial.print(calib.accelBias[0]);
    Serial.print(", ");
    Serial.print(calib.accelBias[1]);
    Serial.print(", ");
    Serial.println(calib.accelBias[2]);
    Serial.println("Gyro biases X/Y/Z: ");
    Serial.print(calib.gyroBias[0]);
    Serial.print(", ");
    Serial.print(calib.gyroBias[1]);
    Serial.print(", ");
    Serial.println(calib.gyroBias[2]);
    if (IMU.hasMagnetometer()) {
      Serial.println("Mag biases X/Y/Z: ");
      Serial.print(calib.magBias[0]);
      Serial.print(", ");
      Serial.print(calib.magBias[1]);
      Serial.print(", ");
      Serial.println(calib.magBias[2]);
      Serial.println("Mag Scale X/Y/Z: ");
      Serial.print(calib.magScale[0]);
      Serial.print(", ");
      Serial.print(calib.magScale[1]);
      Serial.print(", ");
      Serial.println(calib.magScale[2]);
    }
    delay(5000);
    IMU.init(calib, IMU_ADDRESS);
  #endif

    //err = IMU.setGyroRange(500);      //USE THESE TO SET THE RANGE, IF AN INVALID RANGE IS SET IT WILL RETURN -1
    //err = IMU.setAccelRange(2);       //THESE TWO SET THE GYRO RANGE TO ±500 DPS AND THE ACCELEROMETER RANGE TO ±2g
    
    if (err != 0) {
      Serial.print("Error Setting range: ");
      Serial.println(err);
      while (true) {
        ;
      }
    }
}

void loop() {
  IMU.update();
  IMU.getAccel(&accelData);
  Serial.print(accelData.accelX);
  Serial.print("\t");
  Serial.print(accelData.accelY);
  Serial.print("\t");
  Serial.print(accelData.accelZ);
  Serial.print("\t");
  IMU.getGyro(&gyroData);
  Serial.print(gyroData.gyroX);
  Serial.print("\t");
  Serial.print(gyroData.gyroY);
  Serial.print("\t");
  Serial.print(gyroData.gyroZ);
  if (IMU.hasMagnetometer()) {
    IMU.getMag(&magData);
    Serial.print("\t");
    Serial.print(magData.magX);
    Serial.print("\t");
    Serial.print(magData.magY);
    Serial.print("\t");
    Serial.print(magData.magZ);
  }
  if (IMU.hasTemperature()) {
	  Serial.print("\t");
	  Serial.println(IMU.getTemp());
  }
  delay(50);
}
