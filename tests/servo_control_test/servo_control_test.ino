#include <Touchstone.h>
#include <ModulatedServo.h>
#include <PID.h>
#include <math.h>

#define SER 4
#define CLK 15
#define RCLK 2

#define servoPin 5

#define targetPort 0

const float unitsPerRadian = 1/PI;

const float encoderTarget = 12;
const float p = -0.2;
const float i = 0;
const float d = -0.1;
const float iCap = 0;
PID pid = PID(p, i, d, iCap);

// MagEncoder object
MagEncoder encoder = MagEncoder();

unsigned long timeStart = 0;

void driveServo(float power) {
  ModulatedServo::drive(power);
}

float prevError = 0;
float sumError = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) {

  }
  
	ModulatedServo::attach(servoPin);

  pid.setOutputRange(0, 1);
  pid.setStepTime(20);
  
  BusChain::begin(SER, CLK, RCLK, 1);
  uint8_t err = BusChain::selectPort(targetPort);
  if (err != 0) {
    Serial.print("Error selecting I2C port: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }
  
  if (encoder.begin()) {
    Serial.println("Error initializing encoder");
    while (true) {
        ;
    }
  }
  encoder.setUnitsPerRadian(unitsPerRadian);
  encoder.setDirection(-1);
  Serial.println("Servo Control Test");

  driveServo(0.05);
  timeStart = millis();
  while (millis() - timeStart < 2000) {
    if (millis() - timeStart < 1000) {
      encoder.calibrate();
    } else {
      encoder.update();
    }
  }
  driveServo(0);
  delay(500);
  encoder.reset();
  pid.reset();
  timeStart = 0;
}

void loop() {
  encoder.update();
  driveServo(pid.update(encoder.relativePosition() - encoderTarget));
  Serial.println(encoder.relativePosition());
}