#include <Arduino.h>

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

// ======================= MAZE DATA =======================

#define MAX_ROWS 9
#define MAX_COLS 9

// Directions: E, N, W, S
const int DX[4] = {0, -1, 0, 1};
const int DY[4] = {1, 0, -1, 0};
const int DIRS[4] = {4, 2, 1, 8}; // bitmask walls

// Movement costs
const float costF1 = 1.2;
const float costF2 = 2.1;
const float costF3 = 3.0;
const float costF4 = 4.0;
const float costTurn90 = 0.6;
const float costTurn180 = 1.1;

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

void stopping()
{
  stopMotors();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////

struct Node
{
  byte x, y, dir;
  float g, f;
  short parent;
  bool opened, closed;
};

Node nodes[MAX_ROWS * MAX_COLS * 4];
int openList[MAX_ROWS * MAX_COLS * 4];
int openCount;

// Global path strings for both mazes
String path9 = "";
String path4 = "";

// ---------- Helper Functions ----------
float heuristic(int x, int y, int gx, int gy)
{
  return abs(gx - x) + abs(gy - y);
}

void addToOpenList(int idx)
{
  int i = openCount++;
  openList[i] = idx;
  while (i > 0)
  {
    int p = (i - 1) / 2;
    if (nodes[openList[p]].f <= nodes[openList[i]].f)
      break;
    int tmp = openList[p];
    openList[p] = openList[i];
    openList[i] = tmp;
    i = p;
  }
}

int popFromOpenList()
{
  int res = openList[0];
  openList[0] = openList[--openCount];
  int i = 0;
  while (true)
  {
    int l = i * 2 + 1, r = l + 1, smallest = i;
    if (l < openCount && nodes[openList[l]].f < nodes[openList[smallest]].f)
      smallest = l;
    if (r < openCount && nodes[openList[r]].f < nodes[openList[smallest]].f)
      smallest = r;
    if (smallest == i)
      break;
    int tmp = openList[i];
    openList[i] = openList[smallest];
    openList[smallest] = tmp;
    i = smallest;
  }
  return res;
}

bool wallPresent(int *maze, int rows, int cols, int x, int y, int dirBit)
{
  if (x < 0 || y < 0 || x >= rows || y >= cols)
    return true;
  return (maze[x * cols + y] & dirBit);
}

// ---------- A* Pathfinder ----------
String findPath(int *maze, int rows, int cols,
                int startX, int startY, int goalX, int goalY, int startDir)
{
  int total = rows * cols * 4;
  for (int i = 0; i < total; i++)
  {
    nodes[i].opened = nodes[i].closed = false;
    nodes[i].parent = -1;
  }
  openCount = 0;

  int startIdx = (startX * cols + startY) * 4 + startDir;
  Node &start = nodes[startIdx];
  start.x = startX;
  start.y = startY;
  start.dir = startDir;
  start.g = 0;
  start.f = heuristic(startX, startY, goalX, goalY);
  addToOpenList(startIdx);
  start.opened = true;

  int found = -1;

  while (openCount > 0)
  {
    int curIdx = popFromOpenList();
    Node &cur = nodes[curIdx];
    cur.closed = true;

    if (cur.x == goalX && cur.y == goalY)
    {
      found = curIdx;
      break;
    }

    // Turns
    for (int t = -2; t <= 2; t++)
    {
      if (t == 0)
        continue;
      int ndir = (cur.dir + t + 4) % 4;
      float turnCost = (abs(t) == 2) ? costTurn180 : costTurn90;
      int nidx = (cur.x * cols + cur.y) * 4 + ndir;
      if (nodes[nidx].closed)
        continue;
      float ng = cur.g + turnCost;
      if (!nodes[nidx].opened || ng < nodes[nidx].g)
      {
        nodes[nidx] = {cur.x, cur.y, (byte)ndir, ng, ng + heuristic(cur.x, cur.y, goalX, goalY),
                       (short)curIdx, true, false};
        addToOpenList(nidx);
      }
    }

    // Forward moves (1–4)
    for (int step = 1; step <= 4; step++)
    {
      int nx = cur.x + DX[cur.dir] * step;
      int ny = cur.y + DY[cur.dir] * step;
      if (nx < 0 || ny < 0 || nx >= rows || ny >= cols)
        break;
      if (wallPresent(maze, rows, cols,
                      cur.x + DX[cur.dir] * (step - 1),
                      cur.y + DY[cur.dir] * (step - 1),
                      DIRS[cur.dir]))
        break;

      float moveCost;
      if (step == 1)
        moveCost = costF1;
      else if (step == 2)
        moveCost = costF2;
      else if (step == 3)
        moveCost = costF3;
      else
        moveCost = costF4;

      int nidx = (nx * cols + ny) * 4 + cur.dir;
      if (nodes[nidx].closed)
        continue;
      float ng = cur.g + moveCost;
      if (!nodes[nidx].opened || ng < nodes[nidx].g)
      {
        nodes[nidx] = {(byte)nx, (byte)ny, cur.dir, ng,
                       ng + heuristic(nx, ny, goalX, goalY),
                       (short)curIdx, true, false};
        addToOpenList(nidx);
      }
    }
  }

  if (found == -1)
    return "No path found";

  // Reconstruct path and encode
  String moves = "";
  int idx = found;
  int lastDir = nodes[idx].dir;
  int lastX = nodes[idx].x, lastY = nodes[idx].y;

  while (nodes[idx].parent != -1)
  {
    Node &p = nodes[nodes[idx].parent];
    if (p.x == lastX && p.y == lastY)
    {
      int diff = (lastDir - p.dir + 4) % 4;
      if (diff == 1)
        moves += 'L';
      else if (diff == 3)
        moves += 'R';
      else if (diff == 2)
        moves += 'U';
      lastDir = p.dir;
    }
    else
    {
      int dx = abs(p.x - lastX) + abs(p.y - lastY);
      // Forward encoding rule
      if (dx == 2)
        moves += 'F';
      else if (dx == 3)
        moves += "FF";
      else if (dx == 4)
        moves += "FFF";
    }
    idx = nodes[idx].parent;
    lastX = p.x;
    lastY = p.y;
  }

  // Reverse string before return
  String reversed = "";
  for (int i = moves.length() - 1; i >= 0; i--)
    reversed += moves[i];
  return reversed;
}

// West 1 North 2 East 4 South 8
int maze9[9][9] = {
    {0xB, 0x6, 0x3, 0x6, 0x3, 0xA, 0x2, 0xA, 0xE},
    {0x7, 0x9, 0x4, 0x5, 0x5, 0x3, 0x0, 0xA, 0xE},
    {0x5, 0x3, 0xC, 0x1, 0xC, 0x5, 0x9, 0xA, 0x6},
    {0x1, 0x4, 0x3, 0xC, 0x7, 0x9, 0x2, 0x6, 0xD},
    {0xD, 0x5, 0x9, 0xA, 0x0, 0xA, 0x4, 0x9, 0xE},
    {0x3, 0x4, 0xB, 0x6, 0x1, 0xE, 0x1, 0xA, 0x6},
    {0x5, 0x9, 0x6, 0x1, 0x8, 0x6, 0x5, 0x3, 0x4},
    {0x9, 0x6, 0x9, 0xC, 0x3, 0x4, 0x5, 0x5, 0xD},
    {0xB, 0xC, 0xB, 0x2, 0xC, 0x9, 0xC, 0x9, 0xE}};

int maze4[4][4] = {
    {0x5, 0x3, 0xA, 0x6},
    {0x9, 0x4, 0xB, 0x4},
    {0x3, 0x4, 0x7, 0x5},
    {0xD, 0x9, 0x8, 0xC}};

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
    return;
  }
  if (moveCount <= moveIteration)
  {
    if (moveCount = path4.length())
    {
      moveForward(0); // Moving foward
    }
    else
    {
      currentMode = STOPPING;
    }
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

  Serial.println("Calculating paths...");

  // East 0 ; North 1 ; West 2 ; South 3
  path9 = findPath((int *)maze9, 9, 9, 0, 0, 8, 8, 0);
  path4 = findPath((int *)maze4, 4, 4, 2, 2, 0, 0, 3);

  Serial.print("9x9 Path: ");
  Serial.println(path9);
  Serial.print("4x4 Path: ");
  Serial.println(path4);
  moveOrder = path4;
  moveCount = moveOrder.length();

  Serial.println("4x4 Maze Solving Started");
}

// ======================= MAIN LOOP =======================
void loop()
{
  long distLeft = getDistance(TRIG_LEFT, ECHO_LEFT) - sensorCorrection;
  long distRight = getDistance(TRIG_RIGHT, ECHO_RIGHT);

  if (distLeft > 60 && distRight > 60 && currentMode == MAZE_SOLVER)
  {
    Serial.println("Line Followowing Started");
    currentMode = LINE_FOLLOWER;
  }
  else if (distLeft < sideThreshold && distRight < sideThreshold && currentMode == LINE_FOLLOWER)
  {
    Serial.println("9x9 Maze Solving Started");
    moveOrder = path9;
    moveCount = moveOrder.length();
    moveIteration = 0;
    currentMode = MAZE_SOLVER;
  }

  if (currentMode == LINE_FOLLOWER)
    lineFollowerLoop();
  else if (currentMode == MAZE_SOLVER)
    mazeSolverLoop();
  else if (currentMode == STOPPING)
    stopping();
}