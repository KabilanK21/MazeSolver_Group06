#include <Arduino.h>

// --- Motor driver pins ---
#define RPWM_L 6
#define LPWM_L 7
#define RPWM_R 10
#define LPWM_R 9

// --- Motor driver enable pins ---
#define R_EN_L 22
#define L_EN_L 23
#define R_EN_R 24
#define L_EN_R 25

// --- Encoder pins ---
#define ENCO_A_L 11
#define ENCO_B_L 12
#define ENCO_A_R 2
#define ENCO_B_R 3

// --- Ultrasonic sensor pins ---
#define TRIG_FRONT 32
#define ECHO_FRONT 33
#define TRIG_LEFT 30
#define ECHO_LEFT 31
#define TRIG_RIGHT 36
#define ECHO_RIGHT 37

// --- Variables ---
volatile long encoderCountLeft = 0;
volatile long encoderCountRight = 0;

int baseSpeedL = 60;
int baseSpeedR = 60;

unsigned long lastTime = 0;
unsigned long interval = 1000;

// --- Encoder ISRs ---
void readEncoderLeft() {
  int b = digitalRead(ENCO_B_L);
  if (b == HIGH) encoderCountLeft++;
  else encoderCountLeft--;
}

void readEncoderRight() {
  int b = digitalRead(ENCO_B_R);
  if (b == HIGH) encoderCountRight++;
  else encoderCountRight--;
}

// --- Motor Control ---
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  // ✅ Adjusted for correct polarity (LEFT motor forward = RPWM_L)
  if (leftSpeed >= 0) {
    analogWrite(RPWM_L, leftSpeed);
    analogWrite(LPWM_L, 0);
  } else {
    analogWrite(RPWM_L, 0);
    analogWrite(LPWM_L, -leftSpeed);
  }

  // ✅ Adjusted for correct polarity (RIGHT motor forward = LPWM_R)
  if (rightSpeed >= 0) {
    analogWrite(LPWM_R, rightSpeed);
    analogWrite(RPWM_R, 0);
  } else {
    analogWrite(LPWM_R, 0);
    analogWrite(RPWM_R, -rightSpeed);
  }
}

void stopMotors() {
  analogWrite(RPWM_L, 0);
  analogWrite(LPWM_L, 0);
  analogWrite(RPWM_R, 0);
  analogWrite(LPWM_R, 0);
}

// --- Ultrasonic Sensor Function ---
long getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);
  long distance = duration * 0.034 / 2;
  if (distance == 0) distance = 300;
  return distance;
}

// --- Setup ---
void setup() {
  Serial.begin(9600);

  pinMode(RPWM_L, OUTPUT);
  pinMode(LPWM_L, OUTPUT);
  pinMode(RPWM_R, OUTPUT);
  pinMode(LPWM_R, OUTPUT);

  pinMode(R_EN_L, OUTPUT);
  pinMode(L_EN_L, OUTPUT);
  pinMode(R_EN_R, OUTPUT);
  pinMode(L_EN_R, OUTPUT);

  digitalWrite(R_EN_L, HIGH);
  digitalWrite(L_EN_L, HIGH);
  digitalWrite(R_EN_R, HIGH);
  digitalWrite(L_EN_R, HIGH);

  pinMode(ENCO_A_L, INPUT_PULLUP);
  pinMode(ENCO_B_L, INPUT_PULLUP);
  pinMode(ENCO_A_R, INPUT_PULLUP);
  pinMode(ENCO_B_R, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCO_A_L), readEncoderLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCO_A_R), readEncoderRight, RISING);

  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);

  Serial.println("Maze Robot Ready...");
  lastTime = millis();
}

// --- Movement helpers ---
void moveForwardForTime(int speedL, int speedR, int durationMs) {
  setMotorSpeeds(speedL, speedR);
  delay(durationMs);
  stopMotors();
}

void turnRight90(int speed, int durationMs) {
  analogWrite(LPWM_L, speed);
  analogWrite(RPWM_L, 0);
  analogWrite(LPWM_R, 0);
  analogWrite(RPWM_R, speed);
  delay(durationMs);
  stopMotors();
}

void turnLeft90(int speed, int durationMs) {
  analogWrite(LPWM_L, 0);
  analogWrite(RPWM_L, speed);
  analogWrite(LPWM_R, speed);
  analogWrite(RPWM_R, 0);
  delay(durationMs);
  stopMotors();
}

// --- Main Loop ---
void loop() {
  long distFront = getDistance(TRIG_FRONT, ECHO_FRONT);
  long distLeft = getDistance(TRIG_LEFT, ECHO_LEFT);
  long distRight = getDistance(TRIG_RIGHT, ECHO_RIGHT);

  Serial.print("F:");
  Serial.print(distFront);
  Serial.print(" L:");
  Serial.print(distLeft);
  Serial.print(" R:");
  Serial.println(distRight);

  // --- Case 1: Front and left blocked ---
  if (distFront < 10 && distLeft < 10) {
    Serial.println("Front & Left blocked -> move forward and turn right");
    stopMotors();
    delay(200);
    moveForwardForTime(40, 400, 400);
    turnRight90(40, 600);
  }

  // --- Case 2: Front only blocked ---
  else if (distFront < 7) {
    stopMotors();
    delay(800);
    if (distRight > 25) turnRight90(40, 600);
    else if (distLeft > 25) turnLeft90(40, 600);
    else turnRight90(40, 600); // dead end
  }

  // --- Case 3: Path correction ---
  else if (distFront > 20){
    long diff = distLeft - distRight;
    int correction = diff * 1.5;
    correction = constrain(correction, -30, 30);

    int adjLeft = baseSpeedL - correction;
    int adjRight = baseSpeedR + correction;

    adjLeft = constrain(adjLeft, 0, 255);
    adjRight = constrain(adjRight, 0, 255);

    setMotorSpeeds(adjLeft, adjRight);
  }

  // --- Print encoder data every second ---
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= interval) {
    noInterrupts();
    long leftCount = encoderCountLeft;
    long rightCount = encoderCountRight;
    encoderCountLeft = 0;
    encoderCountRight = 0;
    interrupts();

    Serial.print("Encoders -> L:");
    Serial.print(leftCount);
    Serial.print("  R:");
    Serial.println(rightCount);

    lastTime = currentTime;
  }
}
