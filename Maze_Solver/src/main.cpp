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
const int IR1 = 40; // Line sensor 1
const int IR2 = 41; // Line sensor 2
const int IR3 = 42; // Line sensor 3 (center)
const int IR4 = 43; // Line sensor 4
const int IR5 = 44; // Line sensor 5

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
void readLineSensors(); // Reads all IR line sensors

// Encoder setup and reading
void setupEncoders();        // Initializes motor encoders
long getLeftEncoderCount();  // Returns current left encoder tick count
long getRightEncoderCount(); // Returns current right encoder tick count
void resetEncoders();        // Resets both encoder tick counters to zero

void setup()
{
    // Initialization code goes here (e.g., pinMode setup, Serial.begin, etc.)
}

void loop()
{
    // Main control loop goes here (movement, sensor checks, etc.)
}
