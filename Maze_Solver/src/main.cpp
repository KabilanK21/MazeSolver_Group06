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
const int R_EN_R = 24;  // Forward enable pin for right motor driver
const int L_EN_R = 25;  // Reverse enable pin for right motor driver

// --- IR line sensor pins ---
const int IR_PINS[8] = {40, 41, 42, 43, 44, 45, 46, 47} // IR Sensors

// --- Function prototypes ---
// Movement control
void moveForward(int SPEED){   // Moves the robot forward
  analogWrite(RPWM_L, SPEED);
  analogWrite(RPWM_R, SPEED);
  analogWrite(LPWM_L, 0);
  analogWrite(LPWM_R, 0);
} 

void turnLeft(int SPEED){  // Turns the robot left
  analogWrite(RPWM_R, SPEED);
  analogWrite(LPWM_R, 0);
  analogWrite(LPWM_L, SPEED);
  analogWrite(RPWM_L, 0);
}   

void turnRight(int SPEED){     // Turns the robot right
  analogWrite(LPWM_R, SPEED);
  analogWrite(RPWM_R, 0);
  analogWrite(RPWM_L, SPEED);
  analogWrite(LPWM_L, 0);
} 

void stopMotors(){    // Stops both motors
  analogWrite(RPWM_L, 0);
  analogWrite(LPWM_L, 0);
  analogWrite(RPWM_R, 0);
  analogWrite(LPWM_R, 0);
}  

// Line sensor reading
void readLineSensors(int sensors[]){ // Reads all IR line sensors
  for (int i = 0; i < 8; i++){
    sensors[i] = digitalRead(IR_PINS[i]);  
  }
}

// --- Continuous Line Following ---
void lineFollowContinuous(){
  int sensors[8];
  static int lastError = 0;

  while (true){   
    readLineSensors(sensors);

    int pos = 0, count = 0;
    for (int i = 0; i < 8; i++){
      if (sensors[i] == 0){ 
        pos += i * 100;
        count++;
      }
    }
  
    if (count > 0){
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
    else{
      stopMotors();
      break; 
    }
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

// Encoder setup and reading
void setupEncoders();        // Initializes motor encoders
long getLeftEncoderCount();  // Returns current left encoder tick count
long getRightEncoderCount(); // Returns current right encoder tick count
void resetEncoders();        // Resets both encoder tick counters to zero

int distance_A, distance_B;  

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
  distance_LEFT = getDistance(TRIG_PIN_LEFT, ECHO_PIN_LEFT);
  delay(50); 
  distance_RIGHT = getDistance(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT);
  delay(50); 
  distance_FRONT = getDistance(TRIG_PIN_FRONT, ECHO_PIN_FRONT);
  delay(50); 

  int diff = abs(distance_LEFT - distance_RIGHT);
  int speedVal = map(diff, 0, 100, 100, 250);  
  if (speedVal > 250) speedVal = 250;
  if (speedVal < 100) speedVal = 100;

  moveForward(100);
  if (distance_LEFT > 10){
    turnLeft(10);
  }
  if(distance_RIGHT > 10){
    turnRight(10);
  }

  encoderCount = 0;

  digitalWrite(R_EN_R, HIGH);
  digitalWrite(L_EN_R, HIGH);
  digitalWrite(R_EN_L, HIGH);
  digitalWrite(L_EN_L, HIGH);

//   int sensors[8];                // Array to hold sensor values
//   readLineSensors(sensors);      // Read values into array

//   // Example: use center sensors for decision
//   if (sensors[3] == 0 && sensors[4] == 0) {
//     moveForward();
//   } else if (sensors[0] == 0 || sensors[1] == 0 || sensors[2] == 0) {
//     turnLeft();
//   } else if (sensors[5] == 0 || sensors[6] == 0 || sensors[7] == 0) {
//     turnRight();
//   } else {
//     stopMotors();
//   }

  lineFollowContinuous();  // Robot will follow line automatically
  delay(2000);

}