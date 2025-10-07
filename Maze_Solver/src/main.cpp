#include <Arduino.h>

// --- Ultrasonic sensor pins ---
#define TRIG_PIN_FRONT 32
#define ECHO_PIN_FRONT 33
#define TRIG_PIN_LEFT 30
#define ECHO_PIN_LEFT 31
#define TRIG_PIN_RIGHT 36
#define ECHO_PIN_RIGHT 37

// --- Encoder pins ---
#define ENCO_A_L 2
#define ENCO_B_L 3
#define ENCO_A_R 11
#define ENCO_B_R 12

// --- Motor driver pins ---
const int RPWM_L = 5;
const int LPWM_L = 6;
const int RPWM_R = 7;
const int LPWM_R = 8;

const int R_EN_L = 22;
const int L_EN_L = 23;
const int R_EN_R = 24;
const int L_EN_R = 25;

// --- IR line sensor pins ---
const int IR_PINS[8] = {40, 41, 42, 43, 44, 45, 46, 47};
const int IR_ENABLE_PIN = 48;

// --- Encoder Variables ---
volatile long encoderCount_L = 0;
volatile long encoderCount_R = 0;
const int TICKS_PER_REV = 220;

// --- Ultrasonic Distances ---
int distance_FRONT = 0;
int distance_LEFT = 0;
int distance_RIGHT = 0;

// ------------------ Function Prototypes ------------------
void moveForward(int SPEED);
void turnLeft(int SPEED);
void turnRight(int SPEED);
void stopMotors();
void readLineSensors(int sensors[]);
void lineFollowContinuous();
int getDistance(int trigPin, int echoPin);
void encoderISR_L();
void encoderISR_R();

// ------------------ Encoder ISRs ------------------
void encoderISR_L()
{
  encoderCount_L++;
}

void encoderISR_R()
{
  encoderCount_R++;
}

// ------------------ Motor Control ------------------
void moveForward(int SPEED)
{
  analogWrite(RPWM_L, SPEED);
  analogWrite(LPWM_L, 0);
  analogWrite(RPWM_R, SPEED);
  analogWrite(LPWM_R, 0);
}

void turnLeft(int SPEED)
{
  analogWrite(RPWM_L, 0);
  analogWrite(LPWM_L, SPEED);
  analogWrite(RPWM_R, SPEED);
  analogWrite(LPWM_R, 0);
}

void turnRight(int SPEED)
{
  analogWrite(RPWM_L, SPEED);
  analogWrite(LPWM_L, 0);
  analogWrite(RPWM_R, 0);
  analogWrite(LPWM_R, SPEED);
}

void stopMotors()
{
  analogWrite(RPWM_L, 0);
  analogWrite(LPWM_L, 0);
  analogWrite(RPWM_R, 0);
  analogWrite(LPWM_R, 0);
}

// ------------------ Line Sensor Reading ------------------
void readLineSensors(int sensors[])
{
  for (int i = 0; i < 8; i++)
  {
    sensors[i] = digitalRead(IR_PINS[i]);
  }
}

// ------------------ Ultrasonic Distance ------------------
int getDistance(int trigPin, int echoPin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 20000);
  int distance = duration * 0.034 / 2;

  return distance;
}

// ------------------ Line Following ------------------
void lineFollowContinuous()
{
  int sensors[8];
  static int lastError = 0;

  readLineSensors(sensors);

  int pos = 0, count = 0;
  for (int i = 0; i < 8; i++)
  {
    if (sensors[i] == 0)
    { // Line detected (black line)
      pos += i * 100;
      count++;
    }
  }

  if (count > 0)
  {
    int avg = pos / count;
    int error = avg - 350;

    float Kp = 0.2, Kd = 1.0;
    int derivative = error - lastError;
    int correction = Kp * error + Kd * derivative;

    int base = 150;
    int leftPWM = base - correction;
    int rightPWM = base + correction;

    analogWrite(RPWM_L, constrain(leftPWM, 0, 255));
    analogWrite(LPWM_L, 0);
    analogWrite(RPWM_R, constrain(rightPWM, 0, 255));
    analogWrite(LPWM_R, 0);

    lastError = error;
  }
  else
  {
    stopMotors();
  }

  // Print IR sensor values for debugging
  Serial.print("IR Sensors: ");
  for (int i = 0; i < 8; i++)
  {
    Serial.print(sensors[i]);
    Serial.print("\t");
  }
  Serial.println();
}

// ------------------ Setup ------------------
void setup()
{
  Serial.begin(9600);

  // Ultrasonic pins
  pinMode(TRIG_PIN_FRONT, OUTPUT);
  pinMode(ECHO_PIN_FRONT, INPUT);
  pinMode(TRIG_PIN_LEFT, OUTPUT);
  pinMode(ECHO_PIN_LEFT, INPUT);
  pinMode(TRIG_PIN_RIGHT, OUTPUT);
  pinMode(ECHO_PIN_RIGHT, INPUT);

  // IR sensors
  for (int i = 0; i < 8; i++)
  {
    pinMode(IR_PINS[i], INPUT);
  }
  pinMode(IR_ENABLE_PIN, OUTPUT);
  digitalWrite(IR_ENABLE_PIN, HIGH);

  // Motor driver
  pinMode(RPWM_L, OUTPUT);
  pinMode(LPWM_L, OUTPUT);
  pinMode(RPWM_R, OUTPUT);
  pinMode(LPWM_R, OUTPUT);
  pinMode(R_EN_L, OUTPUT);
  pinMode(L_EN_L, OUTPUT);
  pinMode(R_EN_R, OUTPUT);
  pinMode(L_EN_R, OUTPUT);

  // Enable motor drivers
  digitalWrite(R_EN_L, HIGH);
  digitalWrite(L_EN_L, HIGH);
  digitalWrite(R_EN_R, HIGH);
  digitalWrite(L_EN_R, HIGH);

  // Encoder interrupts
  attachInterrupt(digitalPinToInterrupt(ENCO_A_L), encoderISR_L, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCO_A_R), encoderISR_R, RISING);
}

// ------------------ Main Loop ------------------
void loop()
{
  // --- Ultrasonic Sensor Readings ---
  distance_LEFT = getDistance(TRIG_PIN_LEFT, ECHO_PIN_LEFT);
  delay(50);
  distance_RIGHT = getDistance(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT);
  delay(50);
  distance_FRONT = getDistance(TRIG_PIN_FRONT, ECHO_PIN_FRONT);
  delay(50);

  // --- Print Ultrasonic Readings ---
  Serial.print("Front: ");
  Serial.print(distance_FRONT);
  Serial.print(" cm | Left: ");
  Serial.print(distance_LEFT);
  Serial.print(" cm | Right: ");
  Serial.print(distance_RIGHT);
  Serial.println(" cm");

  // --- Example Movement Logic ---
  if (distance_FRONT < 10)
  {
    stopMotors();
    Serial.println("Obstacle Ahead!");
  }
  else if (distance_LEFT < 10)
  {
    turnRight(150);
  }
  else if (distance_RIGHT < 10)
  {
    turnLeft(150);
  }
  else
  {
    moveForward(150);
  }

  // --- Uncomment to enable line following ---
  // lineFollowContinuous();

  delay(200);
}
