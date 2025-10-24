#include <Arduino.h>

const int SIZE = 4;
const int MAX_NODES = 64;

// ======================= IR LINE FOLLOWER CONFIG =======================
const int IR_PINS[8] = {41, 37, 36, 33, 32, 31, 30, 28};
const int IR_ENABLE = 40;

// PID constants
float Kp = 0.27;
float Ki = 0.0;
float Kd = 0.13;

// ======================= SPEED CONTROLS =======================
#define FORWARD_SPEED 120
#define TURN_SPEED 120
#define LINE_SPEED 120
float forwardSpeedFactor = 0.6;
float turnSpeedFactor = 0.4;
float lineSpeedFactor = 0.7;
int lastLeftSpeed = 0;
int lastRightSpeed = 0;

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
#define ENCO_A_L 18
#define ENCO_B_L 12
#define ENCO_A_R 2
#define ENCO_B_R 3
volatile long encoderCountLeft = 0;
volatile long encoderCountRight = 0;

// ======================= ULTRASONIC CONFIG =======================
#define TRIG_FRONT 48
#define ECHO_FRONT 49
#define TRIG_LEFT 42
#define ECHO_LEFT 43
#define TRIG_RIGHT 46
#define ECHO_RIGHT 47

// ======================= MAZE NAVIGATION CONFIG =======================
int frontThreshold = 11;
int sideThreshold = 15;
float correctionGain = 2.5; // Forward Movement Left & Right Adjustment
long countsFor90Deg = 130;
int targetWallDist = 6;
long preTurnClearance = 110;
long postTurnClearance = 120;
long oneCellCount = 300;
int sensorCorrection = 97;
String moveOrder = "";
int moveCount = 0;
int moveIteration = 0;

// ======================= LINE FOLLOWING CONFIG =======================

float TURN_GAIN = 1.6;

// ======================= PRINT CONTROL =======================
unsigned long lastPrintTime = 0;
const unsigned long printInterval = 500;

// ======================= MODE CONTROL =======================
enum Mode
{
  LINE_FOLLOWER,
  MAZE_SOLVER,
  STOPPING
};
Mode currentMode = MAZE_SOLVER; // Start in maze solver mode

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
  lastLeftSpeed = leftSpeed;
  lastRightSpeed = rightSpeed;
}

void stopMotors()
{
  int step = 7;
  for (int s = max(lastLeftSpeed, lastRightSpeed); s > 0; s -= step)
  {
    setMotorSpeeds(s, s);
    delay(20);
  }
  setMotorSpeeds(0, 0);
}

void stopMotorsSudden()
{
  int step = 12;
  for (int s = max(lastLeftSpeed, lastRightSpeed); s > 0; s -= step)
  {
    setMotorSpeeds(s, s);
    delay(20);
  }

  // Step 2: Small reverse pulse to brake
  int reverseSpeed = -50;
  setMotorSpeeds(reverseSpeed, reverseSpeed);
  delay(400);

  setMotorSpeeds(0, 0);
}

// ======================= ULTRASONIC MEASUREMENT =======================
long getDistance(int trigPin, int echoPin)
{
  const int N = 3;
  long sum = 0;
  for (int i = 0; i < N; i++)
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
    sum += distance;
    delay(2);
  }
  return sum / N;
}

// ======================= MAZE SOLVER LOGIC =======================
void moveForward(int typeRun) // 0 - Normal, 1 - Pre Turn Clearance, 2 - Post Turn Clearance, 3 - One cell
{
  noInterrupts();
  long startRight = encoderCountRight;
  long startLeft = encoderCountLeft;
  interrupts();

  int adjLeft, adjRight;

  while (true)
  {
    // Update sensor readings every loop
    long distFront = getDistance(TRIG_FRONT, ECHO_FRONT) - sensorCorrection;
    long distLeft = getDistance(TRIG_LEFT, ECHO_LEFT) - sensorCorrection;
    long distRight = getDistance(TRIG_RIGHT, ECHO_RIGHT);

    adjLeft = FORWARD_SPEED * forwardSpeedFactor;
    adjRight = FORWARD_SPEED * forwardSpeedFactor;

    // --- Wall following corrections ---
    if (distFront < frontThreshold)
    {
      break;
    }
    else if (distLeft < sideThreshold && distRight < sideThreshold)
    {
      long diff = distLeft - distRight;
      int correction = constrain(diff * correctionGain, -30, 30);
      adjLeft -= correction;
      adjRight += correction;
    }
    else if (distLeft < sideThreshold && distRight >= sideThreshold)
    {
      long error = distLeft - targetWallDist;
      int correction = constrain(error * correctionGain, -30, 30);
      adjLeft -= correction;
      adjRight += correction;
    }
    else if (distRight < sideThreshold && distLeft >= sideThreshold)
    {
      long error = distRight - targetWallDist;
      int correction = constrain(error * correctionGain, -30, 30);
      adjLeft += correction;
      adjRight -= correction;
    }

    // Constrain motor values
    adjLeft = constrain(adjLeft, 0, 255);
    adjRight = constrain(adjRight, 0, 255);

    // Apply speeds

    setMotorSpeeds(adjLeft, adjRight);

    // --- Check distance moved ---
    noInterrupts();
    long rightMoved = abs(encoderCountRight - startRight);
    long leftMoved = abs(encoderCountLeft - startLeft);
    interrupts();
    if (typeRun == 1)
    {
      if ((rightMoved + leftMoved) / 2 >= preTurnClearance)
        break;
    }
    else if (typeRun == 2)
    {
      if ((rightMoved + leftMoved) / 2 >= postTurnClearance)
        break;
    }
    else if (typeRun == 3)
    {
      if ((rightMoved + leftMoved) / 2 >= oneCellCount)
        break;
    }
    else
      break;

    delay(5); // Small delay to avoid extremely fast loop causing overshoot
  }
}

void turnRight90()
{
  int speed = TURN_SPEED * turnSpeedFactor;

  noInterrupts();
  long startLeft = encoderCountLeft;
  long startRight = encoderCountRight;
  interrupts();

  setMotorSpeeds(speed, -speed);

  while (true)
  {
    noInterrupts();
    long leftMoved = abs(encoderCountLeft - startLeft);
    long rightMoved = abs(encoderCountRight - startRight);
    interrupts();

    if (leftMoved + rightMoved >= countsFor90Deg * 2)
      break;
  }
}

void turnLeft90()
{
  int speed = TURN_SPEED * turnSpeedFactor;

  // --- Step 1: Perform the right turn ---
  noInterrupts();
  long startLeft = encoderCountLeft;
  long startRight = encoderCountRight;
  interrupts();

  setMotorSpeeds(-speed, speed);

  while (true)
  {
    noInterrupts();
    long leftMoved = abs(encoderCountLeft - startLeft);
    long rightMoved = abs(encoderCountRight - startRight);
    interrupts();

    if (leftMoved + rightMoved >= countsFor90Deg * 2)
      break;
  }
}

void turnAround()
{
  stopMotors();
  int speed = TURN_SPEED * turnSpeedFactor;
  noInterrupts();
  long startLeft = encoderCountLeft;
  long startRight = encoderCountRight;
  interrupts();

  setMotorSpeeds(-speed, speed);
  while (true)
  {
    noInterrupts();
    long leftMoved = abs(encoderCountLeft - startLeft);
    long rightMoved = abs(encoderCountRight - startRight);
    interrupts();
    if (leftMoved + rightMoved >= countsFor90Deg * 4)
      break;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////

struct Node
{
  int r, c, dir;
  float cost;
  String path;
  bool used;
};

Node openList[MAX_NODES];
int openCount = 0;
float best[SIZE][SIZE][4];

// Directions: 0=North,1=East,2=South,3=West
int dr[4] = {-1, 0, 1, 0};
int dc[4] = {0, 1, 0, -1};

// Wall bits: West(1), North(2), East(4), South(8)
bool canMove(int maze[SIZE][SIZE], int r, int c, int dir)
{
  // Mapping directions (for bit lookups)
  int dirBits[4] = {2, 4, 8, 1};      // N,E,S,W
  int oppositeBits[4] = {8, 1, 2, 4}; // opposite of N,E,S,W

  // Check current cell
  if (maze[r][c] & dirBits[dir])
    return false;

  int nr = r + dr[dir];
  int nc = c + dc[dir];
  if (nr < 0 || nr >= SIZE || nc < 0 || nc >= SIZE)
    return false;

  // Check target cell's opposite wall
  if (maze[nr][nc] & oppositeBits[dir])
    return false;

  return true;
}

String moveDir(int prev, int next)
{
  int diff = (next - prev + 4) % 4;
  if (diff == 0)
    return "F";
  if (diff == 1)
    return "R";
  if (diff == 3)
    return "L";
  return "U";
}

int findLowestCost()
{
  int bestIdx = -1;
  float bestCost = 1e9;
  for (int i = 0; i < openCount; i++)
  {
    if (!openList[i].used && openList[i].cost < bestCost)
    {
      bestCost = openList[i].cost;
      bestIdx = i;
    }
  }
  return bestIdx;
}

void addNode(int r, int c, int dir, float cost, const String &path)
{
  if (openCount < MAX_NODES)
  {
    openList[openCount] = {r, c, dir, cost, path, false};
    openCount++;
  }
}

void findFastestPath(int maze[SIZE][SIZE],
                     int startR, int startC,
                     int endR, int endC,
                     int initialFacing,
                     float straight_costs[4],
                     float turn90, float turn180)
{
  for (int r = 0; r < SIZE; r++)
    for (int c = 0; c < SIZE; c++)
      for (int d = 0; d < 4; d++)
        best[r][c][d] = 1e9;

  openCount = 0;
  addNode(startR, startC, initialFacing, 0.0, "");
  best[startR][startC][initialFacing] = 0.0;

  while (true)
  {
    int idx = findLowestCost();
    if (idx == -1)
      break;

    Node current = openList[idx];
    openList[idx].used = true;

    if (current.r == endR && current.c == endC)
    {
      Serial.println("Fastest Path (Robot View):");
      Serial.println(current.path);
      Serial.print("Total Cost: ");
      Serial.println(current.cost, 2);
      moveOrder = current.path;
      moveCount = moveOrder.length();
      return;
    }

    for (int ndir = 0; ndir < 4; ndir++)
    {
      if (!canMove(maze, current.r, current.c, ndir))
        continue;

      int nr = current.r + dr[ndir];
      int nc = current.c + dc[ndir];
      float newCost = current.cost + straight_costs[ndir];

      int diff = abs(current.dir - ndir);
      if (diff == 2)
        newCost += turn180;
      else if (diff == 1 || diff == 3)
        newCost += turn90;

      if (newCost < best[nr][nc][ndir])
      {
        best[nr][nc][ndir] = newCost;
        String move = moveDir(current.dir, ndir);
        String newPath = current.path;
        newPath += move;
        addNode(nr, nc, ndir, newCost, newPath);
      }
    }
  }
  Serial.println("No path found!");
}

void stopping()
{
  stopMotors();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

// ======================= LINE FOLLOWING LOGIC =======================
void lineFollowerLoop()
{
  int sensors[8];
  int sum = 0, blackCount = 0;

  for (int i = 0; i < 8; i++)
  {
    sensors[i] = digitalRead(IR_PINS[i]);
    if (sensors[i] == 1) // Assuming 1 means "on the line" (black)
    {
      sum += i * 100;
      blackCount++;
    }
  }

  // --- LINE LOST (No line/All white) ---
  if (blackCount == 0)
  {
    stopMotorsSudden();
    if (sensors[0] == 1)
    {
      turnRight90();
    }
    else
    {
      turnLeft90();
    }
    integral = 0;
    if (millis() - lastPrintTime >= printInterval)
    {
      Serial.println("L turn Detected");
      lastPrintTime = millis();
    }
    delay(10);
    return;
  }

  // --- NORMAL PID LINE FOLLOWING ---
  int position = sum / blackCount;
  error = position - 350; // Error is displacement from center (350)

  integral += error;
  float derivative = error - lastError;
  float correction = (Kp * error + Ki * integral + Kd * derivative) * TURN_GAIN;

  lastError = error;

  int leftSpeed = LINE_SPEED - correction;
  int rightSpeed = LINE_SPEED + correction;

  leftSpeed *= lineSpeedFactor;
  rightSpeed *= lineSpeedFactor;

  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  setMotorSpeeds(leftSpeed, rightSpeed);

  // --- Printing Debug Info ---
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

void mazeSolverLoop()
{
  long distFront = getDistance(TRIG_FRONT, ECHO_FRONT) - sensorCorrection;
  long distLeft = getDistance(TRIG_LEFT, ECHO_LEFT) - sensorCorrection;
  long distRight = getDistance(TRIG_RIGHT, ECHO_RIGHT);

  Serial.print("F:");
  Serial.print(distFront);
  Serial.print(" L:");
  Serial.print(distLeft);
  Serial.print(" R:");
  Serial.println(distRight);

  if (moveCount == 0)
  {
    currentMode = STOPPING;
    Serial.println("Stopping 4");
    return;
  }
  if (moveCount <= moveIteration)
  {
    moveForward(0); // Moving foward
    // currentMode = LINE_FOLLOWER;
    //  currentMode = STOPPING;
    return;
  }

  char move = moveOrder[moveIteration];
  if (move == 'F')
  {
    if (distFront > frontThreshold)
    {
      Serial.println("Moving Forward by 1 Cell");
      moveForward(3);
      moveIteration++;
    }
    else
    {
      Serial.println("Stopping 1");
      currentMode = STOPPING;
    }
  }
  else if (move == 'L')
  {
    if (distLeft > sideThreshold)
    {
      if (moveIteration == 0)
      {
        turnLeft90();
      }
      else
      {
        Serial.println("Turning Left");
        moveForward(1);
        stopMotors();
        turnLeft90();
        moveForward(2);
      }
      moveIteration++;
    }
    else
    {
      moveForward(0);
    }
  }
  else if (move == 'R')
  {
    if (distRight > sideThreshold)
    {
      if (moveIteration == 0)
      {
        turnLeft90();
      }
      else
      {
        Serial.println("Turning Right");
        moveForward(1);
        stopMotors();
        turnRight90();
        moveForward(2);
      }
      moveIteration++;
    }
    else
    {
      moveForward(0);
    }
  }
  else
  {
    if (distFront > frontThreshold)
    {
      Serial.println("Turning Around");
      turnAround();
      moveIteration++;
    }
    else
    {
      moveForward(0);
    }
  }
}

// ======================= SETUP =======================
void setup()
{
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

  pinMode(IR_ENABLE, OUTPUT);
  digitalWrite(IR_ENABLE, HIGH);
  for (int i = 0; i < 8; i++)
    pinMode(IR_PINS[i], INPUT);

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

  Serial.println("Robot Ready — Default Mode: Maze Solver");

  ////////////////////////////////////////////////////////////////////////////////////////////
  int maze[4][4] = {
      {0x5, 0x3, 0xA, 0x6},
      {0x9, 0x4, 0xB, 0x4},
      {0x3, 0x4, 0x7, 0x5},
      {0xD, 0x9, 0x8, 0xC}}; // West = 1, North = 2, East = 4, South = 8

  float straight_costs[4] = {0.0, 1.2, 2.1, 3.0};
  findFastestPath(maze, 3, 0, 0, 0, 0, straight_costs, 0.6, 1.1); // Directions: 0=North,1=East,2=South,3=West
}

// ======================= MAIN LOOP =======================
void loop()
{
  long distLeft = getDistance(TRIG_LEFT, ECHO_LEFT) - sensorCorrection;
  long distRight = getDistance(TRIG_RIGHT, ECHO_RIGHT);

  if (distLeft > 60 && distRight > 60 && currentMode == MAZE_SOLVER)
  {
    currentMode = LINE_FOLLOWER;
    Serial.println("Line Followowing Started");
  }
  else if (distLeft < sideThreshold && distRight < sideThreshold && (currentMode == LINE_FOLLOWER || currentMode == STOPPING))
  {
    currentMode = MAZE_SOLVER;
  }

  if (currentMode == LINE_FOLLOWER)
    lineFollowerLoop();
  else if (currentMode == MAZE_SOLVER)
    mazeSolverLoop();
  else if (currentMode == STOPPING)
    stopping();
}