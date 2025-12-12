#include <Arduino.h>
#include <EEPROM.h>

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
int frontThreshold = 10;
int sideThreshold = 15;
float correctionGain = 3.5; // Forward Movement Left & Right Adjustment
long countsFor90Deg = 127;
int targetWallDist = 6;
long oneCellCount = 260;
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

int lastR = -1;
int lastC = -1;
int lastDir = -1;
int endPointR = -1;
int endPointC = -1;
bool loadedMap4 = false;
bool loadedMap9 = false;
bool loadedVar = false;

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
void moveForward(long forwardCount) // 0 - Normal
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
    if (forwardCount > 0)
    {
      if ((rightMoved + leftMoved) / 2 >= forwardCount)
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
  {
    currentMode = STOPPING;
    return "No path found";
  }

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
      for (int i = 0; i < dx; i++)
        moves += 'F';
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

// Wrapper: copy a uint8_t map into temporary int array and call findPath
String findPathFromUint8(uint8_t *map, int rows, int cols,
                         int startX, int startY, int goalX, int goalY, int startDir)
{
  // allocate on stack if small
  int tmp = 0; // placeholder to avoid unused warning in some toolchains
  int total = rows * cols;
  int *buf = (int *)malloc(sizeof(int) * total);
  if (!buf)
    return String("No memory");
  for (int r = 0; r < rows; r++)
    for (int c = 0; c < cols; c++)
      buf[r * cols + c] = (int)map[r * cols + c];
  String res = findPath(buf, rows, cols, startX, startY, goalX, goalY, startDir);
  free(buf);
  return res;
}

// West 1 North 2 East 4 South 8
int maze9[9][9] = {
    {0xA, 0x6, 0x3, 0x6, 0x3, 0xA, 0x2, 0xA, 0xE},
    {0x7, 0x9, 0x4, 0x5, 0x5, 0x3, 0x0, 0xA, 0xE},
    {0x5, 0x3, 0xC, 0x1, 0xC, 0x5, 0x9, 0xA, 0x6},
    {0x1, 0x4, 0x3, 0xC, 0x7, 0x9, 0x2, 0x6, 0xD},
    {0xD, 0x5, 0x9, 0xA, 0x0, 0xA, 0x4, 0x9, 0xE},
    {0x3, 0x4, 0xB, 0x6, 0x1, 0xE, 0x1, 0xA, 0x6},
    {0x5, 0x9, 0x6, 0x1, 0x8, 0x6, 0x5, 0x3, 0x4},
    {0x9, 0x6, 0x9, 0xC, 0x3, 0x4, 0x5, 0x5, 0xD},
    {0xB, 0xC, 0xB, 0xA, 0xC, 0x9, 0xC, 0x9, 0xE}};

int maze4[4][4] = {
    {0x5, 0x3, 0xA, 0x6},
    {0x9, 0x4, 0xB, 0x4},
    {0x3, 0x4, 0x7, 0x5},
    {0xD, 0x9, 0x8, 0xC}};

///////////////////////////////////////////////////////////////////////////////////////////////////

// ======================= EXPLORATION STORAGE (stored in RAM; optionally persisted to EEPROM) ======
uint8_t exploredMaze4[4][4];
uint8_t exploredMaze9[9][9];

// EEPROM layout / constants for Arduino Uno
const int EEPROM_MAGIC_ADDR = 0;   // uint16_t (2 bytes)
const int EEPROM_VERSION_ADDR = 2; // uint8_t
const int EEPROM_MAP4_ADDR = 4;    // 16 bytes
const int EEPROM_MAP9_ADDR = 20;   // 81 bytes
const int EEPROM_VAR1_ADDR = 101;
const int EEPROM_VAR2_ADDR = 103;
const uint16_t EEPROM_MAGIC = 0xA5A5;
const uint8_t EEPROM_VERSION = 1;

// Save/load explored maps to EEPROM (use update to avoid unnecessary writes)
void saveMap4ToEEPROM(uint8_t *map4, int rows4, int cols4)
{
  // header
  EEPROM.update(EEPROM_MAGIC_ADDR, (uint8_t)(EEPROM_MAGIC & 0xFF));
  EEPROM.update(EEPROM_MAGIC_ADDR + 1, (uint8_t)((EEPROM_MAGIC >> 8) & 0xFF));
  EEPROM.update(EEPROM_VERSION_ADDR, EEPROM_VERSION);

  int addr = EEPROM_MAP4_ADDR;
  for (int r = 0; r < rows4; r++)
    for (int c = 0; c < cols4; c++)
      EEPROM.update(addr++, map4[r * cols4 + c]);
}

void saveMap9ToEEPROM(uint8_t *map9, int rows9, int cols9)
{
  // header
  EEPROM.update(EEPROM_MAGIC_ADDR, (uint8_t)(EEPROM_MAGIC & 0xFF));
  EEPROM.update(EEPROM_MAGIC_ADDR + 1, (uint8_t)((EEPROM_MAGIC >> 8) & 0xFF));
  EEPROM.update(EEPROM_VERSION_ADDR, EEPROM_VERSION);

  int addr = EEPROM_MAP9_ADDR;
  for (int r = 0; r < rows9; r++)
    for (int c = 0; c < cols9; c++)
      EEPROM.update(addr++, map9[r * cols9 + c]);
}

void saveVariablesToEEPROM(uint16_t var1, uint16_t var2)
{
  // Save var1 (16-bit)
  EEPROM.update(EEPROM_VAR1_ADDR, (uint8_t)(var1 & 0xFF));
  EEPROM.update(EEPROM_VAR1_ADDR + 1, (uint8_t)((var1 >> 8) & 0xFF));

  // Save var2 (16-bit)
  EEPROM.update(EEPROM_VAR2_ADDR, (uint8_t)(var2 & 0xFF));
  EEPROM.update(EEPROM_VAR2_ADDR + 1, (uint8_t)((var2 >> 8) & 0xFF));
}

bool loadMap4FromEEPROM(uint8_t *map4, int rows4, int cols4)
{
  uint16_t magic = EEPROM.read(EEPROM_MAGIC_ADDR) | (EEPROM.read(EEPROM_MAGIC_ADDR + 1) << 8);
  if (magic != EEPROM_MAGIC)
    return false;
  // optional: check version
  int addr = EEPROM_MAP4_ADDR;
  for (int r = 0; r < rows4; r++)
    for (int c = 0; c < cols4; c++)
      map4[r * cols4 + c] = EEPROM.read(addr++);
  return true;
}

bool loadMap9FromEEPROM(uint8_t *map9, int rows9, int cols9, int var1, int var2)
{
  uint16_t magic = EEPROM.read(EEPROM_MAGIC_ADDR) | (EEPROM.read(EEPROM_MAGIC_ADDR + 1) << 8);
  if (magic != EEPROM_MAGIC)
    return false;
  // optional: check version
  int addr = EEPROM_MAP9_ADDR;
  for (int r = 0; r < rows9; r++)
    for (int c = 0; c < cols9; c++)
      map9[r * cols9 + c] = EEPROM.read(addr++);
  var1 = EEPROM.read(EEPROM_VAR1_ADDR);
  var2 = EEPROM.read(EEPROM_VAR2_ADDR);
  return true;
}

void loadVarsFromEEPROM(int var1, int var2)
{
  var1 = EEPROM.read(EEPROM_VAR1_ADDR);
  var2 = EEPROM.read(EEPROM_VAR2_ADDR);
}

// Invalidate saved maps by clearing the magic header (fast, minimal EEPROM writes)
void eraseMapsInvalidateHeader()
{
  EEPROM.update(EEPROM_MAGIC_ADDR, 0xFF);
  EEPROM.update(EEPROM_MAGIC_ADDR + 1, 0xFF);
}

// mark wall at cell (x,y) in dirBit and set corresponding wall on neighbor for generic sized map
void markWallGeneric(uint8_t *explored, int rows, int cols, int x, int y, int dirBit)
{
  if (x < 0 || y < 0 || x >= rows || y >= cols)
    return;
  explored[x * cols + y] |= dirBit;
  int dirIdx = 0;
  for (int i = 0; i < 4; i++)
    if (DIRS[i] == dirBit)
      dirIdx = i;
  int nx = x + DX[dirIdx];
  int ny = y + DY[dirIdx];
  if (nx >= 0 && ny >= 0 && nx < rows && ny < cols)
    explored[nx * cols + ny] |= DIRS[(dirIdx + 2) % 4];
}

// Sense walls around current pose and update explored map (generic)
void senseAndUpdateWallsGeneric(uint8_t *explored, int rows, int cols, int curX, int curY, int curDir)
{
  long df = getDistance(TRIG_FRONT, ECHO_FRONT) - sensorCorrection;
  long dl = getDistance(TRIG_LEFT, ECHO_LEFT) - sensorCorrection;
  long dr = getDistance(TRIG_RIGHT, ECHO_RIGHT);

  Serial.print("Front: ");
  Serial.print(df);
  Serial.print("   Left: ");
  Serial.print(dl);
  Serial.print("   Right: ");
  Serial.println(dr);

  int frontBit = DIRS[curDir];
  int leftBit = DIRS[(curDir + 1) % 4];
  int rightBit = DIRS[(curDir + 3) % 4];

  if (df <= sideThreshold)
    markWallGeneric(explored, rows, cols, curX, curY, frontBit);
  if (dl <= sideThreshold)
    markWallGeneric(explored, rows, cols, curX, curY, leftBit);
  if (dr <= sideThreshold)
    markWallGeneric(explored, rows, cols, curX, curY, rightBit);
}

// Turn robot to target absolute direction index (0..3) and update dir reference
void turnToDirGeneric(int &curDir, int targetDir)
{
  int diff = (targetDir - curDir + 4) % 4;
  if (diff == 0)
    return;
  else if (diff == 1)
  {
    stopMotors();
    turnLeft90();
  }
  else if (diff == 3)
  {
    stopMotors();
    turnRight90();
  }
  else
    turnAround();
  curDir = targetDir;
}

// Move forward one cell using existing movement and update pos
void moveOneCellForwardUpdatePoseGeneric(int &curX, int &curY, int curDir)
{
  moveForward(oneCellCount);
  curX += DX[curDir];
  curY += DY[curDir];
}

// Generic DFS exploration for small mazes (rows x cols), fills 'explored' flattened array
void exploreMazeGeneric(int rows, int cols, int startX, int startY, int startDir, uint8_t *explored)
{
  int maxCells = rows * cols;
  // temporary visited map
  bool visited[81];
  for (int i = 0; i < maxCells; i++)
  {
    explored[i] = 0;
    visited[i] = false;
  }

  int curX = startX;
  int curY = startY;
  int curDir = startDir;

  // stack for backtracking
  int stackX[81];
  int stackY[81];
  int top = 0;

  visited[curX * cols + curY] = true;

  while (true)
  {
    senseAndUpdateWallsGeneric(explored, rows, cols, curX, curY, curDir);
    Serial.print("At: ");
    Serial.print(curX);
    Serial.print(",");
    Serial.print(curY);
    Serial.print(" Dir: ");
    Serial.println(curDir);

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

    Serial.print("IR: ");
    for (int i = 0; i < 8; i++)
    {
      Serial.print(sensors[i]);
      Serial.print(" ");
    }
    Serial.println();

    // --- LINE LOST (No line/All white) ---
    if (blackCount == 0 && !(curX == 0 && curY == 0) && endPointR == -1)
    {
      endPointR = curX;
      endPointC = curY;
      Serial.print("End Point Detected : (");
      Serial.print(endPointR);
      Serial.print(",");
      Serial.print(endPointC);
      Serial.println(")");
      saveVariablesToEEPROM(endPointC, endPointR);
    }

    // try neighbors in order: front, left, right, back
    bool moved = false;
    int order[4] = {0, 1, 3, 2};
    for (int k = 0; k < 4; k++)
    {
      int ndir = (curDir + order[k]) % 4;
      int nx = curX + DX[ndir];
      int ny = curY + DY[ndir];
      if (nx < 0 || ny < 0 || nx >= rows || ny >= cols)
        continue;
      // if wall present in this direction, skip
      if (explored[curX * cols + curY] & DIRS[ndir])
        continue;
      if (!visited[nx * cols + ny])
      {
        // push current pos to stack for backtracking
        stackX[top] = curX;
        stackY[top] = curY;
        top++;
        // turn, move
        turnToDirGeneric(curDir, ndir);
        moveOneCellForwardUpdatePoseGeneric(curX, curY, curDir);
        visited[curX * cols + curY] = true;
        moved = true;
        break;
      }
    }
    if (moved)
      continue;

    // no unvisited neighbors: backtrack
    if (top == 0)
    {
      lastR = curX;
      lastC = curY;
      lastDir = curDir;
      stopMotors();
      break; // exploration finished
    }

    top--;
    int px = stackX[top];
    int py = stackY[top];
    // compute direction to prev
    int dx = px - curX;
    int dy = py - curY;
    int targetDir = -1;
    for (int d = 0; d < 4; d++)
    {
      if (DX[d] == dx && DY[d] == dy)
      {
        targetDir = d;
        break;
      }
    }
    if (targetDir == -1)
      break; // something wrong
    turnToDirGeneric(curDir, targetDir);
    moveOneCellForwardUpdatePoseGeneric(curX, curY, curDir);
  }
}

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
    if (moveCount == path4.length())
    {
      moveForward(0); // Moving forward
      currentMode = STOPPING;
    }
    else
    {
      // moveForward(oneCellCount);
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
      moveForward(oneCellCount);
    }
    moveIteration++;
  }
  else if (move == 'L')
  {
    if (distLeft > sideThreshold)
    {
      Serial.println("Turning Left");
      stopMotors();
      turnLeft90();
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
      Serial.println("Turning Right");
      stopMotors();
      turnRight90();
      moveIteration++;
    }
    else
    {
      moveForward(0);
    }
  }
  else if (move == 'U')
  {
    Serial.println("Turning Around");
    turnAround();
    moveIteration++;
  }
  else
    currentMode = STOPPING;
}

void solveMaze4()
{
  // Try to load previously saved maps from EEPROM (will fail because header was invalidated)
  loadedMap4 = loadMap4FromEEPROM(&exploredMaze4[0][0], 4, 4);
  if (loadedMap4)
  {
    Serial.println("Loaded explored map4 from EEPROM.");
    Serial.println("Explored 4x4 map (hex):");
    for (int i = 0; i < 4; i++)
    {
      for (int j = 0; j < 4; j++)
      {
        Serial.print("0x");
        Serial.print(exploredMaze4[i][j], HEX);
        Serial.print(" ");
      }
      Serial.println();
    }
    path4 = findPathFromUint8(&exploredMaze4[0][0], 4, 4, 2, 2, 0, 0, 3);
  }
  else
  {
    // Explore 4x4 (East 0 ; North 1 ; West 2 ; South 3)
    Serial.println("Exploring 4x4 maze...");
    exploreMazeGeneric(4, 4, 2, 2, 0, &exploredMaze4[0][0]);
    Serial.println("Explored 4x4 map (hex):");
    for (int i = 0; i < 4; i++)
    {
      for (int j = 0; j < 4; j++)
      {
        Serial.print("0x");
        Serial.print(exploredMaze4[i][j], HEX);
        Serial.print(" ");
      }
      Serial.println();
    }
    saveMap4ToEEPROM(&exploredMaze4[0][0], 4, 4);
    // path4 = findPathFromUint8(&exploredMaze4[0][0], 4, 4, lastR, lastC, endPointR, endPointC, lastDir);
    path4 = findPathFromUint8(&exploredMaze4[0][0], 4, 4, lastR, lastC, 0, 0, lastDir);
  }
  Serial.print("4x4 Discovered Path: ");
  Serial.println(path4);
}

void solveMaze9()
{
  // Try to load previously saved maps from EEPROM (will fail because header was invalidated)
  loadedMap9 = loadMap9FromEEPROM(&exploredMaze9[0][0], 9, 9, endPointR, endPointC);
  if (loadedMap9)
  {
    Serial.println("Loaded explored map9 from EEPROM.");
    Serial.println("Explored 9x9 map (hex):");
    for (int i = 0; i < 9; i++)
    {
      for (int j = 0; j < 9; j++)
      {
        Serial.print("0x");
        Serial.print(exploredMaze9[i][j], HEX);
        Serial.print(" ");
      }
      Serial.println();
    }
    Serial.print("End Point Loaded : (");
    Serial.print(endPointR);
    Serial.print(",");
    Serial.print(endPointC);
    Serial.println(")");
    // path9 = findPathFromUint8(&exploredMaze9[0][0], 9, 9, 0, 0, 8, 8, 0);
    path9 = findPathFromUint8(&exploredMaze9[0][0], 9, 9, 0, 0, endPointR, endPointC, 0);
  }
  else
  {
    // Explore 9X9 (East 0 ; North 1 ; West 2 ; South 3)
    Serial.println("Exploring 9x9 maze...");
    exploreMazeGeneric(9, 9, 0, 0, 0, &exploredMaze9[0][0]);
    Serial.println("Explored 9x9 map (hex):");
    for (int i = 0; i < 9; i++)
    {
      for (int j = 0; j < 9; j++)
      {
        Serial.print("0x");
        Serial.print(exploredMaze9[i][j], HEX);
        Serial.print(" ");
      }
      Serial.println();
    }
    saveMap9ToEEPROM(&exploredMaze9[0][0], 9, 9);
    path9 = findPathFromUint8(&exploredMaze9[0][0], 9, 9, lastR, lastC, endPointR, endPointC, lastDir);
    // path9 = findPathFromUint8(&exploredMaze9[0][0], 9, 9, lastR, lastC, 8, 8, lastDir);
  }
  Serial.print("9x9 Discovered Path: ");
  Serial.println(path9);
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

  eraseMapsInvalidateHeader();

  solveMaze4();

  moveOrder = path4;
  moveCount = moveOrder.length();
}

// ======================= MAIN LOOP =======================
void loop()
{
  long distFront = getDistance(TRIG_FRONT, ECHO_FRONT) - sensorCorrection;
  long distLeft = getDistance(TRIG_LEFT, ECHO_LEFT) - sensorCorrection;
  long distRight = getDistance(TRIG_RIGHT, ECHO_RIGHT);

  if (distLeft > 60 && distRight > 60 && distFront > 20 && currentMode == MAZE_SOLVER)
  {
    Serial.println("Line Followowing Started");
    currentMode = LINE_FOLLOWER;
  }
  else if (distLeft < sideThreshold && distRight < sideThreshold && currentMode == LINE_FOLLOWER)
  {
    Serial.println("9x9 Maze Solving Started");
    solveMaze9();
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