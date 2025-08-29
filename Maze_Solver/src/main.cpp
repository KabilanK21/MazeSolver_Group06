#include <Arduino.h>

#define TRIG_PIN_A 9
#define ECHO_PIN_A 10
#define TRIG_PIN_B 11
#define ECHO_PIN_B 12
#define Enco_A1 2
#define Enco_B1 3
#define Enco_A2 24
#define Enco_B2 25


const int RPWM1 = 5;
const int LPWM1 = 6; 
const int RPWM2 = 26;
const int LPWM2 = 27; 
const int R_EN = 22;
const int L_EN = 23;

const int IR1 = 40;
const int IR2 = 41;
const int IR3 = 42;
const int IR4 = 43;
const int IR5 = 44;

void moveForward();
void turnLeft();
void turnRight();
void stopMotors();

long readFrontUltrasonic();
long readLeftUltrasonic();
long readRightUltrasonic();

void readLineSensors();

void setupEncoders();
long getLeftEncoderCount();
long getRightEncoderCount();
void resetEncoders();

void setup() {
  
}

void loop() {
  
}
