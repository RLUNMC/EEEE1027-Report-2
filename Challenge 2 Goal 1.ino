#include <LiquidCrystal.h>
#include <Wire.h>
#include "I2Cdev.h"
#include <MPU6050.h>
#define sensorLeft A0
#define sensorRight A1
#define encoderLeft A2
#define encoderRight A3
MPU6050 mpu;
int16_t ax, ay, az;
int16_t gx, gy, gz;
float rampAngle;
unsigned long currentTime;
unsigned long lastTime;
float alpha = 0.98;
float deltaTime;
float gyroYrate;
float gyroYangle;
float accelYangle;
float resetThreshold = 7.0;

const int RS = 12, EN = 13, D4 = 8, D5 = 9, D6 = 10, D7 = 11;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);
int ENA = 6, IN1 = 7, IN2 = 4, IN3 = 3, IN4 = 2, ENB = 5;

bool beginning = true;
bool upRamp = false;
bool calibrateAngle = false;
bool rotationStart = false;
bool downRamp = false;
bool downSlope = false;
bool flatSurface = false;
bool finalTrack = false;
unsigned long s1Time;
unsigned long s1EndTime = 5000;
unsigned long s2Time;
unsigned long s2EndTime = 500;
unsigned long s3Time;
unsigned long s3EndTime = 1500;
unsigned long s4PassedTime;
unsigned long s4StartTime;
unsigned long s4Time;
unsigned long s4EndTime = 2000;


float previousRTime = 0;
float currentRTime = 0;
float changeRTime;
float rotationAngle = 0.0;


//To determine the distance travelled by the line-following vehicle.
volatile int countEL; //Count encoder left
volatile int countER; //Count encoder right
int finalEncoderStateL = LOW;
int finalEncoderStateR = LOW;
float circumference = PI * 6.7; //Diameter of each wheels = 6.7cm
int ppr = 20; //Pulse Per Revolution
unsigned long previousTime = 0;
unsigned long displayInterval = 100;
//

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(sensorLeft, INPUT);
  pinMode(sensorRight, INPUT);
  pinMode(encoderLeft, INPUT);
  pinMode(encoderRight, INPUT);

  Wire.begin();
  mpu.initialize();

  lcd.begin(16, 2);
  lcd.print("--Welcome!--");
  delay(1500);
  lcd.clear();
  lcd.print("--Get Ready--");
  delay(1500);
  lcd.clear();
  lastTime = millis();
}

void loop() {
  bool sensorLeftReading = digitalRead(sensorLeft) == HIGH;
  bool sensorRightReading = digitalRead(sensorRight) == HIGH;
  int currentEncoderStateL = digitalRead(encoderLeft);
  int currentEncoderStateR = digitalRead(encoderRight);
  float distanceTravelled = 0.0;

  mpu.getAcceleration(&ax, &ay, &az);
  mpu.getRotation(&gx, &gy, &gz);

  if (abs(ax) < 100 || abs(ay) < 100 || abs(az) < 100) { //Correction in accelerometer readings.
    ax = 0; 
    ay = 0; 
    az = 16384;
  }
  accelYangle = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI; //Y-axis angle calculation.
  if (isnan(accelYangle)) {
    accelYangle = 0.0;
  }
  gyroYrate = gy / 131.0;
  currentTime = millis();
  deltaTime = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;
  gyroYangle += gyroYrate * deltaTime;
  //Apply high-pass filter for gyroscope and low-pass filter for accelerometer.
  rampAngle = alpha * gyroYangle + (1 - alpha) * accelYangle;
  if (isnan(rampAngle)) {
    rampAngle = 0.0;
  }
  if(beginning) {
    if(rampAngle<=25.5) {
      moveStarting(sensorLeftReading, sensorRightReading);
    }
    else if(rampAngle>25.5) {
      s1Time = millis();
      upRamp = true;
      beginning = false;
    }
    displayAngle(rampAngle);
  }

  if(upRamp) {
    if(millis()-s1Time<=s1EndTime) {
      stop();
    }
    else if(millis()-s1Time>s1EndTime) {
      if(!calibrateAngle) {
        if (abs(rampAngle) < resetThreshold) {
          rampAngle = 0.0;
          s2Time = millis();
          calibrateAngle = true;
        }
      }
      if(rampAngle>7.0) {
        moveAverage(sensorLeftReading, sensorRightReading);
      } if(rampAngle<7.0) {
        forwardF();
        delay(500);
        stop();
        delay(4000);
        rotation360();
        stop();
        delay(2000);
        s3Time = millis();
        upRamp = false;
        downRamp = true;
      }
    }
  }

  if(downRamp) {
    if(rampAngle<7.0 && rampAngle>=-5.0) {
      moveStarting(sensorLeftReading, sensorRightReading);
    }
    if(rampAngle<-5.0 && rampAngle>-60.0) {
      downSlope = true;
      downRamp = false;
    }
  }

  if(downSlope) {
    if(rampAngle<-5.0 && rampAngle>-60.0) {
      moveDownRamp(sensorLeftReading, sensorRightReading);
    }
    if(rampAngle>=0.0 && rampAngle<5.0) {
      countEL = 0;
      countER = 0;
      s4StartTime = millis();
      previousTime = millis();
      flatSurface = true;
      downSlope = false;
    }
  }

  if(flatSurface) {
    s4PassedTime = millis() - s4StartTime;
    if (currentEncoderStateL != finalEncoderStateL) {
      if (currentEncoderStateL == HIGH) {
        countEL++;
      }
      finalEncoderStateL = currentEncoderStateL;
    }
    if (currentEncoderStateR != finalEncoderStateR) {
      if (currentEncoderStateR == HIGH) {
        countER++;
      }
      finalEncoderStateR = currentEncoderStateR;
    }

    if (millis() - previousTime >= displayInterval) {
      int countET = countEL + countER;
      distanceTravelled = (float)(countET * circumference * 0.5) / ppr;
      previousTime = millis();
      displayDandT(distanceTravelled, s4PassedTime);
    }

    if(distanceTravelled<=48.0) {
      move(sensorLeftReading, sensorRightReading);
    }
    if(distanceTravelled>48.0) {
      s4Time = millis();
      finalTrack = true;
      flatSurface = false;
    }
    delay(17);
  }

  if(finalTrack) {
    s4PassedTime = millis() - s4StartTime;
    if (currentEncoderStateL != finalEncoderStateL) {
      if (currentEncoderStateL == HIGH) {
        countEL++;
      }
      finalEncoderStateL = currentEncoderStateL;
    }
    if (currentEncoderStateR != finalEncoderStateR) {
      if (currentEncoderStateR == HIGH) {
        countER++;
      }
      finalEncoderStateR = currentEncoderStateR;
    }

    if (millis() - previousTime >= displayInterval) {
      int countET = countEL + countER;
      distanceTravelled = (float)(countET * circumference * 0.5) / ppr;
      previousTime = millis();
      displayDandT(distanceTravelled, s4PassedTime);
    }
    if(millis()-s4Time<=s4EndTime) {
      stop();
    }
    if(millis()-s4Time>s4EndTime) {
      moveTrack(sensorLeftReading, sensorRightReading);
    }
    delay(17);
  }
}

//Defined Functions
void forward() {
  analogWrite(ENA, 95);
  analogWrite(ENB, 95);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void forwardF() {
  analogWrite(ENA, 120);
  analogWrite(ENB, 120);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void stop() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void left() {
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void right() {
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void boost() {
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void avgSpeed() {
  analogWrite(ENA, 200);
  analogWrite(ENB, 200);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void moveStarting(bool sensorLeftReading, bool sensorRightReading) {
  if((sensorLeftReading && sensorRightReading) || (!sensorLeftReading && !sensorRightReading)) {
    forwardF();
  } else if(!sensorLeftReading && sensorRightReading) {
    left();
  } else if(sensorLeftReading && !sensorRightReading) {
    right();
  } 
}

void move(bool sensorLeftReading, bool sensorRightReading) {
  if(sensorLeftReading && sensorRightReading) {
    forward();
  } else if(!sensorLeftReading && sensorRightReading) {
    left();
  } else if(sensorLeftReading && !sensorRightReading) {
    right();
  } else {
    stop();
  }
}

void moveTrack(bool sensorLeftReading, bool sensorRightReading) {
  if(sensorLeftReading && sensorRightReading) {
    forward();
  } else if(!sensorLeftReading && sensorRightReading) {
    left();
  } else if(sensorLeftReading && !sensorRightReading) {
    right();
  } else {
    stop();
  }
}

void moveDownRamp(bool sensorLeftReading, bool sensorRightReading) {
  if((sensorLeftReading && sensorRightReading) || (!sensorLeftReading && !sensorRightReading)) {
    forward();
  } else if(!sensorLeftReading && sensorRightReading) {
    left();
  } else if(sensorLeftReading && !sensorRightReading) {
    right();
  } 
}

void moveAverage(bool sensorLeftReading, bool sensorRightReading) {
  if((sensorLeftReading && sensorRightReading) || (!sensorLeftReading && !sensorRightReading)) {
    avgSpeed();
  } else if(!sensorLeftReading && sensorRightReading) {
    left();
  } else if(sensorLeftReading && !sensorRightReading) {
    right();
  } 
}

void displayAngle(float angle) {
  lcd.setCursor(0, 0);
  lcd.print("A = ");
  lcd.print(angle, 1);
  lcd.print("deg");
}

void displayDandT(float distanceTravelled, unsigned long time) {
  lcd.setCursor(0, 1);
  lcd.print(distanceTravelled, 1); 
  lcd.print("cm,");
  lcd.print("T = ");
  lcd.print(time/1000);
  lcd.print("s");
}

void rotation360() {
  previousRTime = millis();
  while(rotationAngle<358) {
    currentRTime = millis();
    changeRTime = (currentRTime-previousRTime)/1000.0;
    previousRTime = currentRTime;
    mpu.getRotation(&gx, &gy, &gz);
    rotationAngle += gz*-1*changeRTime/131.0;
    delay(20);
  }
  analogWrite(ENA, 200);
  analogWrite(ENB, 180);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}
//
