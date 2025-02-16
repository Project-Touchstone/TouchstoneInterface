#include <ServoController.h>
#include <BusChain.h>
#include <math.h>

#define SER 4
#define CLK 15
#define RCLK 2

#define servoChannel 0
#define servoDriverPort 11
#define interruptPin 5

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
}

void loop() {
  for (float i = -1; i < 1; i+=0.001) {
    ServoController::setPower(servoChannel, i);
    delay(100);
  }
}