#include <LiquidCrystal.h>
#define sensorLeft A2
#define sensorRight A3
#define encoderLeft A4
#define encoderRight A5
#define pi 3.1415926535897932384

const int RS = 8, EN = 9, D4 = 4, D5 = 5, D6 = 6, D7 = 7;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);
int ENA = 3, IN1 = 0, IN2 = 1, IN3 = 2, IN4 = 12, ENB = 11;

volatile int countEL = 0; //Count encoder left
volatile int countER = 0; //Count encoder right
int finalEncoderStateL = LOW;
int finalEncoderStateR = LOW;
float circumference = pi * 6.7; //Diameter of each wheels = 6.7cm
int ppr = 20; //Pulse Per Revolution
unsigned long previousTime = 0;
unsigned long displayInterval = 100;

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

  lcd.begin(16, 2);
  lcd.print("--Welcome!--");
  delay(1500);
  lcd.clear();
  lcd.print("--Get Ready--");
  delay(1500);
  lcd.clear();
  lcd.print("Distance:");
}

void loop() {
  bool sensorLeftReading = digitalRead(sensorLeft) == HIGH;
  bool sensorRightReading = digitalRead(sensorRight) == HIGH;
  int currentEncoderStateL = digitalRead(encoderLeft);
  int currentEncoderStateR = digitalRead(encoderRight);

  if(sensorLeftReading && sensorRightReading) {
    forward();
  } else if(!sensorLeftReading && sensorRightReading) {
    left();
  } else if(sensorLeftReading && !sensorRightReading) {
    right();
  } else {
    stop();
  }

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
    float distanceTravelled = (float)(countET * circumference * 0.5) / ppr;
    lcd.setCursor(0, 1);
    lcd.print(distanceTravelled); 
    lcd.print("cm");
    previousTime = millis();
  }
  delay(20);
}

void forward() {
  analogWrite(ENA, 90);
  analogWrite(ENB, 90);
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
  digitalWrite(IN4,HIGH);
}