#include <Arduino.h>

// --- Ultrasonic sensor pins ---
#define TRIG_PIN_FRONT 32 // Trigger pin for ultrasonic sensor FRONT
#define ECHO_PIN_FRONT 33 // Echo pin for ultrasonic sensor FRONT
#define TRIG_PIN_LEFT 30 // Trigger pin for ultrasonic sensor LEFT
#define ECHO_PIN_LEFT 31 // Echo pin for ultrasonic sensor LEFT
#define TRIG_PIN_RIGHT 34 // Trigger pin for ultrasonic sensor RIGHT
#define ECHO_PIN_RIGHT 35 // Echo pin for ultrasonic sensor RIGHT

// --- Encoder pins ---
#define ENCO_A_L 2  // Left motor encoder channel A
#define ENCO_B_L 3  // Left motor encoder channel B
#define ENCO_A_R 11 // Right motor encoder channel A
#define ENCO_B_R 12 // Right motor encoder channel B

// --- Motor driver pins ---
const int RPWM_L = 5;  // Left motor forward (PWM)
const int LPWM_L = 6;  // Left motor reverse (PWM)
const int RPWM_R = 7; // Right motor forward (PWM)
const int LPWM_R = 8; // Right motor reverse (PWM)
const int R_EN_L = 22;  // Forward enable pin for left motor driver
const int L_EN_L = 23;  // Reverse enable pin for left motor driver
const int R_EN_R = 52;  // Forward enable pin for right motor driver
const int L_EN_R = 53;  // Reverse enable pin for right motor driver

// --- IR line sensor pins ---
const int IR_PINS[8] = {40, 41, 42, 43, 44, 45, 46, 47} // IR Sensors

// --- Function prototypes ---
// Movement control
void moveForward(); // Moves the robot forward
void turnLeft();    // Turns the robot left
void turnRight();   // Turns the robot right
void stopMotors();  // Stops both motors

// Ultrasonic sensor readings
long readFrontUltrasonic(); // Reads distance from front ultrasonic sensor
long readLeftUltrasonic();  // Reads distance from left ultrasonic sensor
long readRightUltrasonic(); // Reads distance from right ultrasonic sensor

// Line sensor reading
void readLineSensors(int sensors[]) { // Reads all IR line sensors
  for (int i = 0; i < 8; i++) {
    sensors[i] = digitalRead(IR_PINS[i]);  
  }
}
// Encoder setup and reading
void setupEncoders();        // Initializes motor encoders
long getLeftEncoderCount();  // Returns current left encoder tick count
long getRightEncoderCount(); // Returns current right encoder tick count
void resetEncoders();        // Resets both encoder tick counters to zero

int distance_A, distance_B;  

volatile long encoderCount = 0;
const int TICKS_PER_REV = 220; // 11 pulses per revolution * 20 (gear ratio)

void encoderISR() {
  encoderCount++;
}

void setup(){
  pinMode(TRIG_PIN_FRONT, OUTPUT);
  pinMode(ECHO_PIN_FRONT, INPUT);

  pinMode(TRIG_PIN_LEFT, OUTPUT);
  pinMode(ECHO_PIN_LEFT, INPUT);

  pinMode(TRIG_PIN_RIGHT, OUTPUT);
  pinMode(ECHO_PIN_RIGHT, INPUT);

  Serial.begin(9600); 

  pinMode(RPWM_L, OUTPUT);
  pinMode(LPWM_L, OUTPUT);
  pinMode(RPWM_R, OUTPUT);
  pinMode(LPWM_R, OUTPUT);

  pinMode(R_EN_L, OUTPUT);
  pinMode(L_EN_L, OUTPUT);
  pinMode(R_EN_R, OUTPUT);
  pinMode(L_EN_R, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(Enco_A1), encoderISR, RISING); 
}

int getDistance(int trigPin, int echoPin){  // FUNCTION FOR READ DISTANCE FORM THE ROBOT
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 20000); 
  int distance = duration * 0.034 / 2;  
  return distance;
}

void loop(){
  distance_Left = getDistance(TRIG_PIN_LEFT, ECHO_PIN_LEFT);
  delay(50); 
  distance_RIGHT = getDistance(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT);
  delay(50); 
  distance_FRONT = getDistance(TRIG_PIN_FRONT, ECHO_PIN_FRONT);
  delay(50); 


  int diff = abs(distance_A - distance_B);
  int speedVal = map(diff, 0, 100, 100, 250);  
  if (speedVal > 250) speedVal = 250;
  if (speedVal < 100) speedVal = 100;

  encoderCount = 0;

  digitalWrite(R_EN_R, HIGH);
  digitalWrite(L_EN_R, HIGH);
  digitalWrite(R_EN_L, HIGH);
  digitalWrite(L_EN_L, HIGH);

  if (distance_A > distance_B) {
    while (encoderCount < TICKS_PER_REV) {
      analogWrite(RPWM1, speedVal);
      analogWrite(LPWM1, 0);
    }
  } 

  if (distance_B > distance_A) {
    while (encoderCount < TICKS_PER_REV) {
      analogWrite(LPWM1, speedVal);
      analogWrite(RPWM1, 0);
    }
  } 

  analogWrite(RPWM1, 0);
  analogWrite(LPWM1, 0);
  digitalWrite(R_EN, LOW);
  digitalWrite(L_EN, LOW);

  delay(1000);

  int sensors[8];                // Array to hold sensor values
  readLineSensors(sensors);      // Read values into array

  // Example: use center sensors for decision
  if (sensors[3] == 0 && sensors[4] == 0) {
    moveForward();
  } else if (sensors[0] == 0 || sensors[1] == 0 || sensors[2] == 0) {
    turnLeft();
  } else if (sensors[5] == 0 || sensors[6] == 0 || sensors[7] == 0) {
    turnRight();
  } else {
    stopMotors();
  }
}