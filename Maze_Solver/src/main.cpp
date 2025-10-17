#include <Arduino.h>

// ======================= IR LINE FOLLOWER CONFIG =======================
const int IR_PINS[8] = {41, 37, 36, 33, 32, 31, 30, 28};
const int IR_ENABLE = 40;

// PID constants
float Kp = 0.25;
float Ki = 0.0;
float Kd = 0.1;
float speedFactor = 0.6;
#define BASE_SPEED 120
#define MAX_SPEED 180
float TURN_GAIN = 1.6;

// PID variables
float error = 0, lastError = 0, integral = 0;

// ======================= MOTOR DRIVER CONFIG =======================
#define RPWM_L 5
#define LPWM_L 6
#define RPWM_R 10
#define LPWM_R 9

#define R_EN_L 22
#define L_EN_L 23
#define R_EN_R 24
#define L_EN_R 25

// ======================= ENCODER CONFIG =======================
#define ENCO_A_L 11
#define ENCO_B_L 12
#define ENCO_A_R 2
#define ENCO_B_R 3
volatile long encoderCountLeft = 0;
volatile long encoderCountRight = 0;

// ======================= ULTRASONIC CONFIG =======================
#define TRIG_FRONT 44
#define ECHO_FRONT 45
#define TRIG_LEFT 42
#define ECHO_LEFT 43
#define TRIG_RIGHT 46
#define ECHO_RIGHT 47

// ======================= MAZE NAVIGATION CONFIG =======================
int baseSpeed = 50;
int frontThreshold = 7;
int sideThreshold = 15;
float correctionGain = 1.5;
long countsFor90Deg = 140;
long turnDelay = 0;
long turnD = 300;
long forwardDelayAfterGap = 400;
unsigned long lastTime = 0;
unsigned long interval = 1000;

// ======================= PRINT CONTROL =======================
unsigned long lastPrintTime = 0;
const unsigned long printInterval = 500;

// ======================= MODE CONTROL =======================43
enum Mode
{
  LINE_FOLLOWER,
  MAZE_SOLVER
};
Mode currentMode = MAZE_SOLVER; // Start in line follower mode

// ======================= ENCODER INTERRUPTS =======================
void readEncoderLeft()
{
  int b = digitalRead(ENCO_B_L);
  encoderCountLeft += (b == HIGH) ? 1 : -1;
}
void readEncoderRight()
{
  int b = digitalRead(ENCO_B_R);
  encoderCountRight += (b == HIGH) ? 1 : -1;
}

// ======================= MOTOR CONTROL =======================
void setMotorSpeeds(int leftSpeed, int rightSpeed)
{
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  // LEFT motor
  if (leftSpeed >= 0)
  {
    analogWrite(RPWM_L, leftSpeed);
    analogWrite(LPWM_L, 0);
  }
  else
  {
    analogWrite(RPWM_L, 0);
    analogWrite(LPWM_L, -leftSpeed);
  }

  // RIGHT motor
  if (rightSpeed >= 0)
  {
    analogWrite(LPWM_R, rightSpeed);
    analogWrite(RPWM_R, 0);
  }
  else
  {
    analogWrite(LPWM_R, 0);
    analogWrite(RPWM_R, -rightSpeed);
  }
}

void stopMotors()
{
  analogWrite(RPWM_L, 0);
  analogWrite(LPWM_L, 0);
  analogWrite(RPWM_R, 0);
  analogWrite(LPWM_R, 0);
}

// ======================= ULTRASONIC MEASUREMENT =======================
long getDistance(int trigPin, int echoPin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 6000);
  long distance = duration * 0.034 / 2;
  if (distance == 0)
    distance = 300;
  return distance;
}

// ======================= LINE FOLLOWING LOGIC =======================
void lineFollowerLoop()
{
  int sensors[8];
  int sum = 0, blackCount = 0;

  for (int i = 0; i < 8; i++)
  {
    sensors[i] = digitalRead(IR_PINS[i]);
    if (sensors[i] == 1)
    {
      sum += i * 100;
      blackCount++;
    }
  }

  if (blackCount == 0)
  {
    stopMotors();
    integral = 0;
    if (millis() - lastPrintTime >= printInterval)
    {
      Serial.println("❌ Line Lost — Motors Stopped");
      lastPrintTime = millis();
    }
    delay(10);
    return;
  }

  int position = sum / blackCount;
  error = position - 350;

  integral += error;
  float derivative = error - lastError;
  float correction = (Kp * error + Ki * integral + Kd * derivative) * TURN_GAIN;

  lastError = error;

  int leftSpeed = BASE_SPEED - correction;
  int rightSpeed = BASE_SPEED + correction;

  leftSpeed *= speedFactor;
  rightSpeed *= speedFactor;

  leftSpeed = constrain(leftSpeed, 0, MAX_SPEED * speedFactor);
  rightSpeed = constrain(rightSpeed, 0, MAX_SPEED * speedFactor);

  setMotorSpeeds(leftSpeed, rightSpeed);

  if (millis() - lastPrintTime >= printInterval)
  {
    Serial.print("IR: ");
    for (int i = 0; i < 8; i++)
    {
      Serial.print(sensors[i]);
      Serial.print(" ");
    }
    Serial.print(" | Pos: ");
    Serial.print(position);
    Serial.print(" | Err: ");
    Serial.print(error);
    Serial.print(" | Corr: ");
    Serial.print(correction);
    Serial.print(" | L: ");
    Serial.print(leftSpeed);
    Serial.print(" | R: ");
    Serial.println(rightSpeed);
    lastPrintTime = millis();
  }
  delay(10);
}

// ======================= MAZE SOLVER LOGIC =======================
void moveForward(long distLeft, long distRight)
{
  // Only apply centering correction if both walls are detected (not a gap)
  if (distLeft < sideThreshold && distRight < sideThreshold)
  {
    long diff = distLeft - distRight;
    int correction = diff * correctionGain;
    correction = constrain(correction, -30, 30);
    int adjLeft = baseSpeed - correction;
    int adjRight = baseSpeed + correction;
    adjLeft = constrain(adjLeft, 0, 255);
    adjRight = constrain(adjRight, 0, 255);
    setMotorSpeeds(adjLeft, adjRight);
  }
  else
  {
    // If one or both sides are open (gap), just move straight
    setMotorSpeeds(baseSpeed, baseSpeed);
  }
}

void turnRight90(int speed)
{
  setMotorSpeeds(30, 30);
  // delay(200);
  noInterrupts();
  long startLeft = encoderCountLeft, startRight = encoderCountRight;
  interrupts();
  setMotorSpeeds(speed, -speed);
  while (true)
  {
    noInterrupts();
    long leftMoved = abs(encoderCountLeft - startLeft);
    long rightMoved = abs(encoderCountRight - startRight);
    interrupts();
    delay(turnDelay);
    if (leftMoved >= countsFor90Deg || rightMoved >= countsFor90Deg)
      break;
  }
  stopMotors();
  // delay(100);
}

void turnLeft90(int speed)
{
  setMotorSpeeds(30, 30);
  // delay(200);
  noInterrupts();
  long startLeft = encoderCountLeft, startRight = encoderCountRight;
  interrupts();
  delay(turnDelay);
  setMotorSpeeds(-speed, speed);
  while (true)
  {
    noInterrupts();
    long leftMoved = abs(encoderCountLeft - startLeft);
    long rightMoved = abs(encoderCountRight - startRight);
    interrupts();
    if (leftMoved >= countsFor90Deg || rightMoved >= countsFor90Deg)
      break;
  }
  stopMotors();
  // delay(100);
}

void turnAround(int speed)
{
  // Rotate ~180 degrees by running wheels opposite until ~2 * countsFor90Deg
  const unsigned long maxTurnMs = 3000; // safety timeout
  long target = countsFor90Deg * 2;
  setMotorSpeeds(30, 30);
  delay(120);
  noInterrupts();
  long startLeft = encoderCountLeft, startRight = encoderCountRight;
  interrupts();
  unsigned long t0 = millis();
  setMotorSpeeds(speed, -speed);
  while (true)
  {
    noInterrupts();
    long leftMoved = abs(encoderCountLeft - startLeft);
    long rightMoved = abs(encoderCountRight - startRight);
    interrupts();
    if (leftMoved >= target || rightMoved >= target)
      break;
    if (millis() - t0 > maxTurnMs)
    {
      Serial.println("turnAround: timeout — stopping turn");
      break;
    }
    delay(1);
  }
  stopMotors();
  // delay(100);
}

void mazeSolverLoop()
{
  long distFront = getDistance(TRIG_FRONT, ECHO_FRONT);
  long distLeft = getDistance(TRIG_LEFT, ECHO_LEFT);
  long distRight = getDistance(TRIG_RIGHT, ECHO_RIGHT);

  Serial.print("F:");
  Serial.print(distFront);
  Serial.print(" L:");
  Serial.print(distLeft);
  Serial.print(" R:");
  Serial.print(distRight);
  Serial.print(" | Speed:");
  Serial.println(baseSpeed);

  if (distFront < frontThreshold)
  {
    stopMotors();
    // delay(100);

    if (distRight > sideThreshold)
    {
      Serial.println("Right gap detected -> move forward before turning...");
      Serial.println("Turning RIGHT 90°...");
      turnRight90(baseSpeed);
    }
    else if (distLeft > sideThreshold)
    {
      Serial.println("Left gap detected -> move forward before turning...");
      Serial.println("Turning LEFT 90°...");
      turnLeft90(baseSpeed);
    }
    else
    {
      Serial.println("Dead end -> Turn RIGHT by default");
      turnAround(baseSpeed);
    }

    delay(200);
  }
  else
  {
    moveForward(distLeft, distRight);
  }

  if (millis() - lastTime >= interval)
  {
    noInterrupts();
    long leftCount = encoderCountLeft;
    long rightCount = encoderCountRight;
    encoderCountLeft = 0;
    encoderCountRight = 0;
    interrupts();

    Serial.print("Encoders -> L:");
    Serial.print(leftCount);
    Serial.print(" R:");
    Serial.println(rightCount);

    lastTime = millis();
  }
}

// ======================= SETUP =======================
void setup()
{
  Serial.begin(9600);

  // Motor setup
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

  // IR setup
  pinMode(IR_ENABLE, OUTPUT);
  digitalWrite(IR_ENABLE, HIGH);
  for (int i = 0; i < 8; i++)
    pinMode(IR_PINS[i], INPUT);

  // Encoder setup
  pinMode(ENCO_A_L, INPUT_PULLUP);
  pinMode(ENCO_B_L, INPUT_PULLUP);
  pinMode(ENCO_A_R, INPUT_PULLUP);
  pinMode(ENCO_B_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCO_A_L), readEncoderLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCO_A_R), readEncoderRight, RISING);

  // Ultrasonic setup
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);

  Serial.println("Robot Ready — Default Mode: Line Follower");
}

// ======================= MAIN LOOP =======================
void loop()
{
  // Read ultrasonic sensors
  long distFront = getDistance(TRIG_FRONT, ECHO_FRONT);
  long distLeft = getDistance(TRIG_LEFT, ECHO_LEFT);
  long distRight = getDistance(TRIG_RIGHT, ECHO_RIGHT);

  // Auto mode switching condition
  if ((distFront > 20 && distLeft > 40) || (distFront > 20 && distRight > 40))
  {
    if (currentMode != LINE_FOLLOWER)
    {
      Serial.println("🔄 Switching to LINE FOLLOWER mode...");
      currentMode = LINE_FOLLOWER;
    }
  }
  else
  {
    if (currentMode != MAZE_SOLVER)
    {
      Serial.println("🔄 Switching to MAZE SOLVER mode...");
      currentMode = MAZE_SOLVER;
    }
  }

  // Run current mode
  if (currentMode == LINE_FOLLOWER)
  {
    lineFollowerLoop();
  }
  else
  {
    mazeSolverLoop();
  }
}
