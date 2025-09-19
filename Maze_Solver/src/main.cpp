#include <Arduino.h>

// --- Ultrasonic sensor pins ---
#define TRIG_PIN_A 9  // Trigger pin for ultrasonic sensor A
#define ECHO_PIN_A 10 // Echo pin for ultrasonic sensor A
#define TRIG_PIN_B 11 // Trigger pin for ultrasonic sensor B
#define ECHO_PIN_B 12 // Echo pin for ultrasonic sensor B

// --- Encoder pins ---
#define Enco_A1 2  // Left motor encoder channel A
#define Enco_B1 3  // Left motor encoder channel B
#define Enco_A2 24 // Right motor encoder channel A
#define Enco_B2 25 // Right motor encoder channel B

// --- Motor driver pins ---
const int RPWM1 = 5;  // Right motor forward (PWM)
const int LPWM1 = 6;  // Right motor reverse (PWM)
const int RPWM2 = 26; // Left motor forward (PWM)
const int LPWM2 = 27; // Left motor reverse (PWM)
const int R_EN = 22;  // Enable pin for right motor driver
const int L_EN = 23;  // Enable pin for left motor driver

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
  pinMode(TRIG_PIN_A, OUTPUT);
  pinMode(ECHO_PIN_A, INPUT);
  pinMode(TRIG_PIN_B, OUTPUT);
  pinMode(ECHO_PIN_B, INPUT);
  Serial.begin(9600); 

  pinMode(RPWM1, OUTPUT);
  pinMode(LPWM1, OUTPUT);
  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(Enco_A1), encoderISR, RISING); 
}

int getDistance(int trigPin, int echoPin){
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
  distance_A = getDistance(TRIG_PIN_A, ECHO_PIN_A);
  delay(50); 
  distance_B = getDistance(TRIG_PIN_B, ECHO_PIN_B);

  int diff = abs(distance_A - distance_B);
  int speedVal = map(diff, 0, 100, 100, 250);  
  if (speedVal > 250) speedVal = 250;
  if (speedVal < 100) speedVal = 100;

  encoderCount = 0;

  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);

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
