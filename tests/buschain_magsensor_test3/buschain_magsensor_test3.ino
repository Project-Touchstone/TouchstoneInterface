#include <Tlv493d.h>
#include <BusChain.h>

#define SER 3
#define CLK 4
#define RCLK 5

#define targetPort 7

byte delaytime = 1;

const uint16_t sensorAddr = 0x5E;
bool speed = true;

uint32_t clockSpeed = 400000;

uint8_t buffer[4];

void convertToBytes(int32_t num) {
  for (int i = 0; i < 4; i++) {
    buffer[i] = (num & 0b11111111);
    num = num >> 8;
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  BusChain.begin(SER, CLK, RCLK, 1, clockSpeed);

  int err = BusChain.selectPort(targetPort);
  if (err != 0) {
    Serial.print("Error selecting I2C port: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }
  TLV493D.begin(sensorAddr, 0);
  //Serial.println("3D Magnetic Sensor Test");
}

//long cycleStart;

void loop() {
  //cycleStart = micros();
 delay(delaytime); // wait time between reads.

  if (TLV493D.update(sensorAddr) == 0) {
        convertToBytes(TLV493D.rawX());
        Serial.write(buffer, 4);
        convertToBytes(TLV493D.rawY());
        Serial.write(buffer, 4);
        convertToBytes(TLV493D.rawZ());
        Serial.write(buffer, 4);
        Serial.write('\n');
  } else {
    //Serial.println("Data read error!");
  }
  //Serial.println(micros()-cycleStart);
}