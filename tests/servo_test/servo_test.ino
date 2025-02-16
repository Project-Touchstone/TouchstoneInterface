#include <ESP32Servo.h>

//Servo pin
#define servoPin 5

Servo servo;

void setup() {
  Serial.begin(9600);
  //Attaches servo
  servo.attach(servoPin);
  servo.write(90);
}

void loop() {
  /*int val = (int) analogRead(pot)*180./800;
  Serial.println(val);
  servo.write(val);*/
}
