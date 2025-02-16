#import "Button.h"
#import <math.h>

Button trigger = Button(2);

uint8_t ser[2] = {3, 4};
#define CLK 5
#define RCLK 6
#define PWM 9
#define POT A0
#define VOUT A1

uint32_t clockInterval = 1000;

int indexLimit = 16;

class ControlParams {
  public:
    ControlParams(uint8_t index, bool polarity) {
      this->index = index;
      this->polarity = polarity;
    };

    uint8_t getIndex() {
      return index;
    }

    bool getPolarity() {
      return polarity;
    }

    void setIndex(uint8_t index) {
      this->index = index;
    }

    void setPolarity(bool polarity) {
      this->polarity = polarity;
    }
  private:
    uint8_t index;
    bool polarity;
};

bool state = false;
bool enabled = false;

ControlParams params[2] = {ControlParams(0, false), ControlParams(0, true)};

uint8_t serBuffers[2] = {0, 0};

uint8_t serRemaining = 0;

uint32_t timerStart = 0;

char strBuffer[32];

int currPWM = 0;
int setPoint = 0;
float gain = 0.01;
int count = 0;

void resetTimer() {
  timerStart = micros();
  state = false;
}

void selectOutput(ControlParams* params) {
  for (uint8_t i = 0; i < 2; i++) {
    ControlParams param = *(params + i);
    serBuffers[i] = param.getPolarity();
    serBuffers[i] += param.getIndex() << 2;
  }

  serRemaining = 5;

  resetTimer();
}

void disableOutput() {
  digitalWrite(PWM, 0);
  currPWM = 0;
  serBuffers[0] = 0;
  serBuffers[1] = 0;
  serRemaining = 5;
  resetTimer();
}

void sendBit() {
  if (serRemaining > 0) {
    for (int i = 0; i < 2; i++) {
      digitalWrite(ser[i], serBuffers[i] & 1);
      serBuffers[i] = serBuffers[i] >> 1;
    }
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

      for (int i = 0; i < 2; i++) {
        digitalWrite(ser[i], 0);
      }
    }
  }
}

void setup() {
  Serial.begin(9600);

  pinMode(CLK, OUTPUT);
  pinMode(RCLK, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(A0, INPUT);

  for (int i = 0; i < 2; i++) {
    pinMode(ser[i], OUTPUT);
  }

  disableOutput();
}

void loop() {
  if (trigger.changeTo(true)) {
    enabled = !enabled;
    if (enabled) {
      selectOutput(params);

      for (int i = 0; i < 2; i++) {
        sprintf(strBuffer, "Index%d: %d; ", i, params[i].getIndex());
        Serial.print(strBuffer);
        sprintf(strBuffer, "Polarity%d: %d; ", i, params[i].getPolarity());
        Serial.println(strBuffer);
      }
      for (int i = 0; i < 2; i++) {
        params[i].setPolarity(!params[i].getPolarity());
      }
      count++;
      if (count > 1) {
        count = 0;
        params[1].setIndex(params[1].getIndex()+1);
      }

      if (params[1].getIndex() >= indexLimit) {
        params[1].setIndex(0);
        params[0].setIndex((params[0].getIndex()+1) % indexLimit);
      }
    } else {
      Serial.println("Disabled");
      disableOutput();
    }
  }
  
  if (enabled && (serRemaining == 0)) {
    /*currPWM += (int) ((setPoint-analogRead(VOUT))*255.0/1023*gain);*/
    currPWM = (int) analogRead(POT)*255.0/860;
    if (currPWM < 0) {
      currPWM = 0;
    } else if (currPWM > 255) {
      currPWM = 255;
    }
    analogWrite(PWM, currPWM);
  }
  
  updateData();
  trigger.update();
}
