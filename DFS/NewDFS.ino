#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MPU6050.h>
#include <ESP32Encoder.h>
#include <Adafruit_VL53L0X.h>
#include <stack>
#include <math.h>
#include <bits/stdc++.h>

// OLED & I2C setup
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// MPU, Encoder, VL53L0X sensors
MPU6050 mpu;
ESP32Encoder encoder1, encoder2;
Adafruit_VL53L0X sensor1, sensor2, sensor3;

#define TCA9548A_ADDR 0x70
#define SCL_PIN 22
#define SDA_PIN 21

// Motor pins
const int PWMA = 12, AIN1 = 33, AIN2 = 32;
const int PWMB = 14, BIN1 = 26, BIN2 = 27;
const int STBY = 25;
#define TIEN 1
#define LUI -1
#define DUNG 0

// Encoder pins
const int encoderPin1_1 = 19, encoderPin2_1 = 18;
const int encoderPin1_2 = 15, encoderPin2_2 = 2;

// Global variables
float yaw = 0;
unsigned long lastTime = 0;
std::stack<std::pair<int,int>> cellStack;
int speed = 160;

// Maze & robot parameters
const int MAZE_SIZE = 16;
const int CELL_SIZE = 160;
const float WHEEL_DIAMETER = 34.0;    // mm
const float ENCODER_TICK_PER_REV = 1386.0; // 1386 xung/vòng
const float WHEEL_BASE = 100.0;       // mm

float robotX = 0, robotY = 0, robotHeading = 0;
long initialEnc1 = 0, initialEnc2 = 0;
float initialHeadingValue = 0;

struct Cell {
  bool visited;
  int order;
  Cell() : visited(false), order(0) {}
};
Cell maze[MAZE_SIZE+5][MAZE_SIZE+5];
int cellOrder = 0;
int currentCellX = -1, currentCellY = -1;

// Sensor thresholds (cm)
const float FORWARD_THRESHOLD = 11.0;
const float LEFT_THRESHOLD = 14.0;
const float RIGHT_THRESHOLD = 18.0;
const int TURN_ANGLE = 90;

// --- Hàm chuyển kênh TCA9548A ---
void tcaSelect(uint8_t channel) {
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// --- Điều khiển động cơ ---
void Left_wheel(int control, int spd) {
  analogWrite(PWMA, spd);
  if(control == TIEN) { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); }
  else if(control == LUI) { digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH); }
  else { digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW); }
}

void Right_wheel(int control, int spd) {
  analogWrite(PWMB, spd);
  if(control == TIEN) { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); }
  else if(control == LUI) { digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); }
  else { digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW); }
}

void stopMovement() {
  Right_wheel(DUNG, 0);
  Left_wheel(DUNG, 0);
}

// --- Hàm chạy thẳng ---
void di_thang(int spd) {
  Right_wheel(TIEN, spd);
  Left_wheel(LUI, spd);
}

// --- Hàm lấy yaw (chỉ giữ lại nếu cần updateRobotPosition) ---
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

// --- Hàm tính hiệu số góc (sử dụng nếu cần tính góc từ robotHeading) ---
float angleDifference(float start, float current) {
  float diff = current - start;
  while(diff > 180) diff -= 360;
  while(diff < -180) diff += 360;
  return fabs(diff);
}

// --- Hàm quay sử dụng encoder (không dùng MPU) ---
// Tính số xung cần thiết để quay một góc nhất định, dựa vào công thức:
// requiredCounts = ( (WHEEL_BASE/2) * (targetAngle * PI/180) / (PI*WHEEL_DIAMETER) ) * 1386
void turnRightEncoder(int spd, float targetAngle) {
  long start1 = encoder1.getCount();
  long start2 = encoder2.getCount();
  float T_rad = targetAngle * PI / 180.0;
  float arcLength = (WHEEL_BASE / 2.0) * T_rad;
  float wheelCircumference = PI * WHEEL_DIAMETER;
  float requiredCounts = (arcLength / wheelCircumference) * 1386.0;
  while(true) {
    long d1 = abs(encoder1.getCount() - start1);
    long d2 = abs(encoder2.getCount() - start2);
    if(d1 + d2 >= requiredCounts) break;
    Right_wheel(TIEN, spd-60);
    Left_wheel(TIEN, spd-60);
  }
  stopMovement();
}

void turnLeftEncoder(int spd, float targetAngle) {
  long start1 = encoder1.getCount();
  long start2 = encoder2.getCount();
  float T_rad = targetAngle * PI / 180.0;
  float arcLength = (WHEEL_BASE / 2.0) * T_rad;
  float wheelCircumference = PI * WHEEL_DIAMETER;
  float requiredCounts = (arcLength / wheelCircumference) * 1386.0;
  while(true) {
    long d1 = abs(encoder1.getCount() - start1);
    long d2 = abs(encoder2.getCount() - start2);
    if(d1 + d2 >= requiredCounts) break;
    Right_wheel(LUI, spd-60);
    Left_wheel(LUI, spd-60);
  }
  stopMovement();
}

// --- Hàm cập nhật vị trí dựa trên encoder ---
void updateRobotPosition() {
  long curEnc1 = encoder1.getCount();
  long curEnc2 = encoder2.getCount();
  long dEnc1 = curEnc1 - initialEnc1;
  long dEnc2 = curEnc2 - initialEnc2;
  float d1 = (dEnc1 / 1386.0) * (PI * WHEEL_DIAMETER);
  float d2 = (dEnc2 / 1386.0) * (PI * WHEEL_DIAMETER);
  float dHeading = (d1 - d2) / WHEEL_BASE;
  // robotHeading vẫn được cập nhật từ encoder (nếu cần)
  robotHeading = initialHeadingValue + dHeading * 180 / PI;
  robotHeading = fmod(robotHeading + 360.0, 360.0);
  float distance = (d1 + d2) / 2.0;
  robotX += distance * cos(robotHeading * PI / 180.0);
  robotY += distance * sin(robotHeading * PI / 180.0);
}

bool isValidCell(int x, int y) {
  return (x >= 0 && x < MAZE_SIZE && y >= 0 && y < MAZE_SIZE);
}

/* DFS Decision:
   Sensor mapping:
     Sensor 1 (forward) > 11 cm,
     Sensor 2 (left) > 14 cm,
     Sensor 3 (right) > 18 cm.
   Priority: Move Forward > Turn Right > Turn Left.
*/
void dfsDecision(int dist_forward, int dist_left, int dist_right) {
  int cellX = (int)((robotX + 1200) / CELL_SIZE);
  int cellY = (int)((robotY + 1200) / CELL_SIZE);
  
  if(cellX != currentCellX || cellY != currentCellY) {
    if(currentCellX != -1 && currentCellY != -1)
      cellStack.push({currentCellX, currentCellY});
    currentCellX = cellX; currentCellY = cellY;
    maze[cellX][cellY].visited = true;
    maze[cellX][cellY].order = ++cellOrder;
    Serial.print("Entered cell: ");
    Serial.print(cellX); Serial.print(", "); Serial.println(cellY);
  }
  
  bool availForward = (dist_forward > 11);
  bool availLeft = (dist_left > 14);
  bool availRight = (dist_right > 18);
  
  float h = robotHeading;
  int nx, ny, lx, ly, rx, ry;
  
  if((h >= 315 || h < 45)) { nx = currentCellX; ny = currentCellY + 1; }
  else if(h >= 45 && h < 135) { nx = currentCellX + 1; ny = currentCellY; }
  else if(h >= 135 && h < 225) { nx = currentCellX; ny = currentCellY - 1; }
  else { nx = currentCellX - 1; ny = currentCellY; }
  bool forwardUnvisited = isValidCell(nx, ny) && !maze[nx][ny].visited;
  
  float hRight = fmod(h + 90, 360);
  if((hRight >= 315 || hRight < 45)) { rx = currentCellX; ry = currentCellY + 1; }
  else if(hRight >= 45 && hRight < 135) { rx = currentCellX + 1; ry = currentCellY; }
  else if(hRight >= 135 && hRight < 225) { rx = currentCellX; ry = currentCellY - 1; }
  else { rx = currentCellX - 1; ry = currentCellY; }
  bool rightUnvisited = isValidCell(rx, ry) && !maze[rx][ry].visited;
  
  float hLeft = fmod(h + 270, 360);
  if((hLeft >= 315 || hLeft < 45)) { lx = currentCellX; ly = currentCellY + 1; }
  else if(hLeft >= 45 && hLeft < 135) { lx = currentCellX + 1; ly = currentCellY; }
  else if(hLeft >= 135 && hLeft < 225) { lx = currentCellX; ly = currentCellY - 1; }
  else { lx = currentCellX - 1; ly = currentCellY; }
  bool leftUnvisited = isValidCell(lx, ly) && !maze[lx][ly].visited;
  
  if(availForward && forwardUnvisited) {
    Serial.println("Decision: Move Forward");
    di_thang(speed);
  } else if(availRight && rightUnvisited) {
    Serial.println("Decision: Turn Right");
    turnRightEncoder(speed, TURN_ANGLE);
    di_thang(speed);
  } else if(availLeft && leftUnvisited) {
    Serial.println("Decision: Turn Left");
    turnLeftEncoder(speed, TURN_ANGLE);
    di_thang(speed);
  } else {
    if(!cellStack.empty()){
      std::pair<int,int> prevCell = cellStack.top();
      cellStack.pop();
      Serial.print("Backtracking to cell: ");
      Serial.print(prevCell.first); Serial.print(", "); Serial.println(prevCell.second);
      int dx = prevCell.first - currentCellX;
      int dy = prevCell.second - currentCellY;
      float targetAngle;
      if(dx == 0 && dy == 1) targetAngle = 0;
      else if(dx == 1 && dy == 0) targetAngle = 90;
      else if(dx == 0 && dy == -1) targetAngle = 180;
      else if(dx == -1 && dy == 0) targetAngle = 270;
      float angleDiff = targetAngle - robotHeading;
      while(angleDiff > 180) angleDiff -= 360;
      while(angleDiff < -180) angleDiff += 360;
      if(angleDiff > 5) {
        turnRightEncoder(speed, angleDiff);
      } else if(angleDiff < -5) {
        turnLeftEncoder(speed, -angleDiff);
      }
      di_thang(speed);
    } else {
      Serial.println("No available move; moving forward.");
      di_thang(speed);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
  
  mpu.initialize();
  if(!mpu.testConnection()){
    Serial.println("MPU6050 connection failed!");
  }
  
  if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)){
    Serial.println("SSD1306 allocation failed!");
  }
  display.clearDisplay();
  
  encoder1.attachHalfQuad(encoderPin1_1, encoderPin2_1);
  encoder2.attachHalfQuad(encoderPin1_2, encoderPin2_2);
  
  // Khởi tạo cảm biến VL53L0X với kênh mới:
  tcaSelect(1); // Sensor 1: đo phía trước
  if(!sensor1.begin()) Serial.println("VL53L0X #1 failed!");
  
  tcaSelect(2); // Sensor 2: đo bên trái
  if(!sensor2.begin()) Serial.println("VL53L0X #2 failed!");
  
  tcaSelect(4); // Sensor 3: đo bên phải
  if(!sensor3.begin()) Serial.println("VL53L0X #3 failed!");
  
  // Cài đặt Bounce2 cho nút bấm
  pinMode(BUTTON_SELECT, INPUT_PULLUP);
  pinMode(BUTTON_BACK, INPUT_PULLUP);
  buttonSelect.attach(BUTTON_SELECT); buttonSelect.interval(10);
  buttonBack.attach(BUTTON_BACK); buttonBack.interval(10);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), readEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), readEncoder, CHANGE);
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.println("Khoi dong...");
  display.display();
  delay(2000);
  
  resetSensors();
  renderMenu();
  
  lastTime = millis();
  initialEnc1 = encoder1.getCount();
  initialEnc2 = encoder2.getCount();
  initialHeadingValue = getYaw();
}

void loop() {
  if(!dfsActive){
    handleEncoder();
    numberDisplay();
    handleButtons();
  } else {
    VL53L0X_RangingMeasurementData_t m0, m1, m2;
    tcaSelect(sensorChannels[0]); // Sensor 1: forward
    delay(10);
    sensor1.rangingTest(&m0, false);
    int dist_forward = m0.RangeMilliMeter / 10;
    
    tcaSelect(sensorChannels[1]); // Sensor 2: left
    delay(10);
    sensor2.rangingTest(&m1, false);
    int dist_left = m1.RangeMilliMeter / 10;
    
    tcaSelect(sensorChannels[2]); // Sensor 3: right
    delay(10);
    sensor3.rangingTest(&m2, false);
    int dist_right = m2.RangeMilliMeter / 10;
    
    updateRobotPosition();
    
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print("Yaw: "); display.println(yaw);
    display.print("Enc: "); display.println(encoder1.getCount());
    display.print("F: "); display.print(dist_forward); display.println(" cm");
    display.print("L: "); display.print(dist_left); display.println(" cm");
    display.print("R: "); display.print(dist_right); display.println(" cm");
    display.display();
    
    Serial.print("Yaw: "); Serial.print(yaw);
    Serial.print(" Enc: "); Serial.print(encoder1.getCount());
    Serial.print(" F: "); Serial.print(dist_forward);
    Serial.print(" cm L: "); Ser
