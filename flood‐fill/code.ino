#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>       
#include <MPU6050.h>
#include <ESP32Encoder.h>
#include <Adafruit_VL53L0X.h>
#include <stack>
#include <queue>
#include <math.h>
#include <bits/stdc++.h>

// ==================== Display and Sensor Setup ====================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

MPU6050 mpu;
ESP32Encoder encoder1, encoder2;
Adafruit_VL53L0X sensor1, sensor2, sensor3;

#define TCA9548A_ADDR 0x70
#define SCL_PIN 22
#define SDA_PIN 21

// ==================== Motor Pins ====================
const int PWMA = 12, AIN1 = 33, AIN2 = 32;  // Left motor
const int PWMB = 14, BIN1 = 26, BIN2 = 27;   // Right motor
const int STBY = 25;
#define TIEN 1   // forward
#define LUI -1   // reverse
#define DUNG 0   // stop

// ==================== Encoder Pins ====================
const int encoderPin1_1 = 19, encoderPin2_1 = 18;
const int encoderPin1_2 = 15, encoderPin2_2 = 2;

// ==================== Global Variables ====================
float yaw = 0;
unsigned long lastTime = 0;

std::stack<std::pair<int,int>> cellStack;  // Stack for DFS backtracking
int speed = 120;

// Maze & robot geometry parameters
const int MAZE_SIZE = 16;        
const int CELL_SIZE = 160;       
const float WHEEL_DIAMETER = 34;  
const float ENCODER_TICK_PER_REV = 107.0;  
const float WHEEL_BASE = 100.0;    

float robotX = 0.0;      
float robotY = 0.0;      
float robotHeading = 0.0; 
long initialEnc1 = 0;
long initialEnc2 = 0;
float initialHeadingValue = 0.0;

// ==================== Maze Cell Structure (with Flood-Fill Value) ====================
struct Cell {
  bool visited;
  int order;          
  int floodFillValue; 
  Cell() : visited(false), order(0), floodFillValue(9999) {} 
};
// Use extra padding (MAZE_SIZE+5) for safety
Cell maze[MAZE_SIZE+5][MAZE_SIZE+5];
int cellOrder = 0;
int currentCellX = -1;
int currentCellY = -1;

// Wall threshold: sensors (after dividing by 10) are in cm, so 3.0 = 3 cm
const float WALL_THRESHOLD = 3.0;  
const int TURN_ANGLE = 90;         // Standard 90° turn

// Define the goal cell for flood fill (here, center of maze)
const int GOAL_X = MAZE_SIZE / 2;
const int GOAL_Y = MAZE_SIZE / 2;

// Robot Modes: Initially, we start in MAPPING mode. When goal reached, switch to PATHFILL.
enum RobotMode { MAPPING, PATHFILL };
RobotMode currentMode = MAPPING;
bool floodFilled = false;  // Indicates if flood fill has been computed

// ==================== Functions for Processing and Calculation ====================

// ----- TCA9548A Channel Selection (unchanged) -----
void tcaSelect(uint8_t channel) {
    Wire.beginTransmission(TCA9548A_ADDR);
    Wire.write(1 << channel);
    Wire.endTransmission();
}

// ----- Motor Control Functions -----
void Left_wheel(int control, int spd) {
    analogWrite(PWMA, spd);
    if (control == TIEN) {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
    } else if (control == LUI) {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
    } else {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, LOW);
    }
}
void Right_wheel(int control, int spd) {
    analogWrite(PWMB, spd);
    if (control == TIEN) {
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);
    } else if (control == LUI) {
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, HIGH);
    } else {
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, LOW);
    }
}

// ----- Get Yaw from MPU6050 -----  
/* 
   dt: time difference in seconds
   gZ: raw gyro value converted to deg/sec (using divisor 131 as per MPU6050 spec)
   yaw: accumulated heading (kept within [0,360) via fmod)
*/
float getYaw() {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0;
    lastTime = currentTime;
    float gZ = gz / 131.0;
    yaw = fmod(yaw + gZ * dt + 360, 360);
    return yaw;
}

// ----- Stop Movement -----
void stopMovement() {
    Right_wheel(DUNG, 0);
    Left_wheel(DUNG, 0);
}

// ----- Drive Straight -----  
/* 
   di_thang: drives forward with a slight PWM reduction (spd-20) for both wheels 
   to help compensate for mechanical differences.
*/
void di_thang(int spd) {
    Right_wheel(TIEN, spd-20);
    Left_wheel(LUI, spd-20);
}

// ----- Angle Difference Helper -----  
/* 
   Computes the absolute difference between two angles, handling wrap-around.
   For example, the difference between 350° and 10° is 20°.
*/
float angleDifference(float start, float current) {
    float diff = current - start;
    while(diff > 180) diff -= 360;
    while(diff < -180) diff += 360;
    return fabs(diff);
}

// ----- Turning Functions -----  
/* 
   turnRight and turnLeft functions:
   - Capture the current yaw (via getYaw)
   - Continue turning until the absolute angle difference reaches targetAngle.
   - A short delay is used for stability.
*/
void turnRight(int spd, float targetAngle, float startYaw) {
    float currentYaw = getYaw();
    while(angleDifference(startYaw, currentYaw) < targetAngle) {
        Right_wheel(TIEN, spd-20);
        Left_wheel(TIEN, spd-20);
        delay(10);
        currentYaw = getYaw();
    }
    stopMovement();
}
void turnLeft(int spd, float targetAngle, float startYaw) {
    float currentYaw = getYaw();
    while(angleDifference(startYaw, currentYaw) < targetAngle) {
        Right_wheel(LUI, spd-20);
        Left_wheel(LUI, spd-20);
        delay(10);
        currentYaw = getYaw();
    }
    stopMovement();
}

// ----- Update Robot Position Using Encoders -----  
/* 
   Calculates the distance each wheel has moved:
   - dist = (delta pulses / pulses per revolution) * (π * WHEEL_DIAMETER)
   - deltaHeading (in radians) is computed from the difference in distances over WHEEL_BASE.
   - robotHeading is updated (converted to degrees) and normalized to [0,360).
   - The average distance is used to update robotX and robotY.
*/
void updateRobotPosition() {
    long currentEnc1 = encoder1.getCount();
    long currentEnc2 = encoder2.getCount();
    long deltaEnc1 = currentEnc1 - initialEnc1;
    long deltaEnc2 = currentEnc2 - initialEnc2;
    float dist1 = (deltaEnc1 / ENCODER_TICK_PER_REV) * PI * WHEEL_DIAMETER;
    float dist2 = (deltaEnc2 / ENCODER_TICK_PER_REV) * PI * WHEEL_DIAMETER;
    float deltaHeading = (dist1 - dist2) / WHEEL_BASE; // in radians
    robotHeading = initialHeadingValue + deltaHeading * 180 / PI;
    robotHeading = fmod(robotHeading + 360.0, 360.0);
    float distance = (dist1 + dist2) / 2.0;
    robotX += distance * cos(robotHeading * PI / 180.0);
    robotY += distance * sin(robotHeading * PI / 180.0);
}

// ----- DFS Decision Function for Maze Mapping -----  
/* 
   DFS Decision:
   1. Compute current cell index (cellX, cellY) using robot position (offset by 1200 mm to ensure positive index).
      - The offset is chosen so that even if robotX or robotY is negative, the resulting index is positive.
   2. If the robot enters a new cell, push the previous cell onto the DFS stack, mark the new cell as visited, and assign it an order.
   3. Map sensor readings (after conversion from mm to cm) to directions:
      - dist2: forward; dist1: right; dist3: left.
      - Using robotHeading (h), the function determines which adjacent cell is forward/right/left:
         • If h is in [315,360) or [0,45): consider robot facing North.
         • If h in [45,135): East; [135,225): South; [225,315): West.
   4. Check if each neighboring cell is valid and unvisited.
   5. Decision priority:
      - If forward cell is unvisited and available (sensor reading > WALL_THRESHOLD), move forward.
      - Else if right cell is unvisited and available, turn right 90° and then move forward.
      - Else if left cell is unvisited and available, turn left 90° and then move forward.
   6. If no unvisited neighbor is available, backtrack:
      - Pop the previous cell from the stack.
      - Compute the required turning angle to face that cell.
      - Turn accordingly and move forward.
*/
void dfsDecision(int dist1, int dist2, int dist3) {
    int cellX = (int)((robotX + 1200) / CELL_SIZE);
    int cellY = (int)((robotY + 1200) / CELL_SIZE);
    
    if(cellX != currentCellX || cellY != currentCellY) {
        if(currentCellX != -1 && currentCellY != -1) {
            cellStack.push({currentCellX, currentCellY});
        }
        currentCellX = cellX;
        currentCellY = cellY;
        maze[cellX][cellY].visited = true;
        maze[cellX][cellY].order = ++cellOrder;  // Đánh số thứ tự ô đã thăm
        Serial.print("Entered cell: ");
        Serial.print(cellX); Serial.print(", "); Serial.println(cellY);
    }
    
    // Sensor mapping: dist2 (forward), dist1 (right), dist3 (left)
    bool availForward = (dist2 > WALL_THRESHOLD);
    bool availRight   = (dist1 > WALL_THRESHOLD);
    bool availLeft    = (dist3 > WALL_THRESHOLD);
    
    float h = robotHeading;
    int nx, ny, rx, ry, lx, ly;
    
    // Determine the forward cell based on heading:
    if ((h >= 315 || h < 45))         { nx = currentCellX;     ny = currentCellY + 1; }
    else if (h >= 45 && h < 135)        { nx = currentCellX + 1; ny = currentCellY; }
    else if (h >= 135 && h < 225)       { nx = currentCellX;     ny = currentCellY - 1; }
    else                              { nx = currentCellX - 1; ny = currentCellY; }
    bool forwardUnvisited = isValidCell(nx, ny) && !maze[nx][ny].visited;
    
    // Determine the right cell (heading + 90°):
    float hRight = fmod(h + 90, 360);
    if ((hRight >= 315 || hRight < 45))  { rx = currentCellX;     ry = currentCellY + 1; }
    else if (hRight >= 45 && hRight < 135){ rx = currentCellX + 1; ry = currentCellY; }
    else if (hRight >= 135 && hRight < 225){ rx = currentCellX;    ry = currentCellY - 1; }
    else                              { rx = currentCellX - 1; ry = currentCellY; }
    bool rightUnvisited = isValidCell(rx, ry) && !maze[rx][ry].visited;
    
    // Determine the left cell (heading - 90° i.e. +270°):
    float hLeft = fmod(h + 270, 360);
    if ((hLeft >= 315 || hLeft < 45))   { lx = currentCellX;     ly = currentCellY + 1; }
    else if (hLeft >= 45 && hLeft < 135)  { lx = currentCellX + 1; ly = currentCellY; }
    else if (hLeft >= 135 && hLeft < 225) { lx = currentCellX;     ly = currentCellY - 1; }
    else                              { lx = currentCellX - 1; ly = currentCellY; }
    bool leftUnvisited = isValidCell(lx, ly) && !maze[lx][ly].visited;
    
    // Decision making: Priority: Forward > Right > Left; else backtrack.
    if (availForward && forwardUnvisited) {
        Serial.println("Decision: Move Forward");
        di_thang(speed);
    } else if (availRight && rightUnvisited) {
        Serial.println("Decision: Turn Right");
        float startYaw = getYaw();
        turnRight(speed, TURN_ANGLE, startYaw);
        di_thang(speed);
    } else if (availLeft && leftUnvisited) {
        Serial.println("Decision: Turn Left");
        float startYaw = getYaw();
        turnLeft(speed, TURN_ANGLE, startYaw);
        di_thang(speed);
    } else {
        // Backtracking: if no unvisited neighbor exists, pop previous cell and turn toward it.
        if (!cellStack.empty()) {
            std::pair<int,int> prevCell = cellStack.top();
            cellStack.pop();
            Serial.print("Backtracking to cell: ");
            Serial.print(prevCell.first); Serial.print(", "); Serial.println(prevCell.second);
            int dx = prevCell.first - currentCellX;
            int dy = prevCell.second - currentCellY;
            float targetAngle;
            if (dx == 0 && dy == 1)       targetAngle = 0;    // North
            else if (dx == 1 && dy == 0)  targetAngle = 90;   // East
            else if (dx == 0 && dy == -1) targetAngle = 180;  // South
            else if (dx == -1 && dy == 0) targetAngle = 270;  // West
            float angleDiff = targetAngle - robotHeading;
            while(angleDiff > 180) angleDiff -= 360;
            while(angleDiff < -180) angleDiff += 360;
            if (angleDiff > 5) {
                float startYaw = getYaw();
                turnRight(speed, angleDiff, startYaw);
            } else if (angleDiff < -5) {
                float startYaw = getYaw();
                turnLeft(speed, -angleDiff, startYaw);
            }
            di_thang(speed);
        } else {
            Serial.println("No available move; moving forward.");
            di_thang(speed);
        }
    }
}

// ----- Flood Fill Algorithm -----  
/* 
   floodFillMaze(goalX, goalY):
   1. Initialize floodFillValue for all cells to a high number (9999), representing "infinity".
   2. Set the floodFillValue of the goal cell to 0.
   3. Use a BFS (queue) to propagate the values to all reachable cells: 
      For each cell, update neighboring cell's value to current cell's value + 1 if it's lower.
*/
void floodFillMaze(int goalX, int goalY) {
    for (int i = 0; i < MAZE_SIZE; i++) {
        for (int j = 0; j < MAZE_SIZE; j++) {
            maze[i][j].floodFillValue = 9999;
        }
    }
    maze[goalX][goalY].floodFillValue = 0;
    
    std::queue<std::pair<int,int>> q;
    q.push({goalX, goalY});
    
    while (!q.empty()) {
        auto current = q.front();
        q.pop();
        int cx = current.first;
        int cy = current.second;
        int currentValue = maze[cx][cy].floodFillValue;
        
        int dx[4] = {0, 1, 0, -1};
        int dy[4] = {1, 0, -1, 0};
        for (int k = 0; k < 4; k++) {
            int nx = cx + dx[k];
            int ny = cy + dy[k];
            if (isValidCell(nx, ny)) {
                if (maze[nx][ny].floodFillValue > currentValue + 1) {
                    maze[nx][ny].floodFillValue = currentValue + 1;
                    q.push({nx, ny});
                }
            }
        }
    }
}

// ----- Follow Flood Fill Gradient -----
/* 
   followFloodFill():
   - For the current cell (currentCellX, currentCellY), check its four neighbors (North, East, South, West).
   - Select the neighbor with the smallest floodFillValue (i.e., the steepest descent toward the goal).
   - Compute the target angle based on the chosen neighbor.
   - Turn toward that angle (using turnRight/turnLeft if needed) and then move forward.
*/
void followFloodFill() {
    int cx = currentCellX;
    int cy = currentCellY;
    int bestValue = 9999;
    int bestDir = -1; // 0: North, 1: East, 2: South, 3: West
    int nx, ny;
    // Check 4 directions
    // North
    if (isValidCell(cx, cy+1) && maze[cx][cy+1].floodFillValue < bestValue) {
        bestValue = maze[cx][cy+1].floodFillValue;
        bestDir = 0;
    }
    // East
    if (isValidCell(cx+1, cy) && maze[cx+1][cy].floodFillValue < bestValue) {
        bestValue = maze[cx+1][cy].floodFillValue;
        bestDir = 1;
    }
    // South
    if (isValidCell(cx, cy-1) && maze[cx][cy-1].floodFillValue < bestValue) {
        bestValue = maze[cx][cy-1].floodFillValue;
        bestDir = 2;
    }
    // West
    if (isValidCell(cx-1, cy) && maze[cx-1][cy].floodFillValue < bestValue) {
        bestValue = maze[cx-1][cy].floodFillValue;
        bestDir = 3;
    }
    float targetAngle;
    // Map bestDir to target angle: 0 = North (0°), 1 = East (90°), 2 = South (180°), 3 = West (270°)
    if (bestDir == 0) targetAngle = 0;
    else if (bestDir == 1) targetAngle = 90;
    else if (bestDir == 2) targetAngle = 180;
    else if (bestDir == 3) targetAngle = 270;
    else targetAngle = robotHeading;  // Fallback
    
    float angleDiff = targetAngle - robotHeading;
    while(angleDiff > 180) angleDiff -= 360;
    while(angleDiff < -180) angleDiff += 360;
    if (angleDiff > 5) {
        float startYaw = getYaw();
        turnRight(speed, angleDiff, startYaw);
    } else if (angleDiff < -5) {
        float startYaw = getYaw();
        turnLeft(speed, -angleDiff, startYaw);
    }
    // After aligning, move forward
    di_thang(speed);
}

// ==================== Setup() and Loop() ====================
// (No detailed comments here as requested)

void setup() {
    Serial.begin(115200);
    Wire.begin(SDA_PIN, SCL_PIN);

    pinMode(PWMA, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(STBY, OUTPUT);
    digitalWrite(STBY, HIGH);

    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed!");
        while (1);
    }

    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println("SSD1306 allocation failed!");
        while (1);
    }
    display.clearDisplay();

    encoder1.attachHalfQuad(encoderPin1_1, encoderPin2_1);
    encoder2.attachHalfQuad(encoderPin1_2, encoderPin2_2);

    tcaSelect(2);
    if (!sensor1.begin()) Serial.println("VL53L0X #1 failed!");
    tcaSelect(6);
    if (!sensor2.begin()) Serial.println("VL53L0X #2 failed!");
    tcaSelect(7);
    if (!sensor3.begin()) Serial.println("VL53L0X #3 failed!");

    lastTime = millis();
    initialEnc1 = encoder1.getCount();
    initialEnc2 = encoder2.getCount();
    initialHeadingValue = getYaw();
}

void loop() {
    VL53L0X_RangingMeasurementData_t measure1, measure2, measure3;
    
    // Read sensor values; divide by 10 to convert mm to cm.
    tcaSelect(2);
    delay(20);
    sensor1.rangingTest(&measure1, false);
    int dist1 = measure1.RangeMilliMeter / 10;

    tcaSelect(6);
    delay(20);
    sensor2.rangingTest(&measure2, false);
    int dist2 = measure2.RangeMilliMeter / 10;

    tcaSelect(7);
    delay(20);
    sensor3.rangingTest(&measure3, false);
    int dist3 = measure3.RangeMilliMeter / 10;
    
    updateRobotPosition();

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print("Yaw: "); display.println(yaw);
    display.print("Enc1: "); display.println(encoder1.getCount());
    display.print("Enc2: "); display.println(encoder2.getCount());
    display.print("D1: "); display.print(dist1); display.println(" cm");
    display.print("D2: "); display.print(dist2); display.println(" cm");
    display.print("D3: "); display.print(dist3); display.println(" cm");
    display.display();

    Serial.print("Yaw: "); Serial.print(yaw);
    Serial.print(" Enc1: "); Serial.print(encoder1.getCount());
    Serial.print(" Enc2: "); Serial.print(encoder2.getCount());
    Serial.print(" D1: "); Serial.print(dist1);
    Serial.print(" cm D2: "); Serial.print(dist2);
    Serial.print(" cm D3: "); Serial.println(dist3);
    
    // Mode switching: Initially use DFS mapping.
    if (currentMode == MAPPING) {
        dfsDecision(dist1, dist2, dist3);
        // Check if robot has reached the goal cell.
        if (currentCellX == GOAL_X && currentCellY == GOAL_Y) {
            Serial.println("Goal reached. Switching to PATHFILL mode.");
            currentMode = PATHFILL;
            floodFilled = false;  
        }
    } else if (currentMode == PATHFILL) {
        if (!floodFilled) {
            floodFillMaze(GOAL_X, GOAL_Y);
            floodFilled = true;
        }
        followFloodFill();
    }
    
    delay(90);
}
