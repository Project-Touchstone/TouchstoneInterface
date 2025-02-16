#include <MagEncoder.h>
#include <BusChain.h>
#include <math.h>

#define SER 4
#define CLK 15
#define RCLK 2

#define targetPort 6

const float unitsPerRadian = 1/PI;
const int8_t direction = -1;

uint64_t startTime;

// Tlv493d Opject
MagEncoder encoder = MagEncoder();

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }
  BusChain::begin(SER, CLK, RCLK, 2);

  int err = BusChain::selectPort(targetPort);
  if (err != 0) {
    Serial.print("Error selecting I2C port: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }
  encoder.begin();
  encoder.setUnitsPerRadian(unitsPerRadian);
  encoder.setDirection(direction);
  Serial.println("Magnetic Encoder Test");

  startTime = millis();
  while (millis() - startTime < 5000) {
    encoder.calibrate();
  }
  encoder.reset();
  Serial.println("Encoder Calibrated");
}

void loop() {
  encoder.update();
  Serial.println(encoder.relativePosition());
}