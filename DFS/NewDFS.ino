#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MPU6050.h>
#include <ESP32Encoder.h>
#include <Adafruit_VL53L0X.h>
#include <Bounce2.h>
#include <stack>
#include <queue>
#include <math.h>
#include <bits/stdc++.h>

// ================= OLED & I2C =================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDR 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ================= MPU, Encoder, VL53L0X =================
MPU6050 mpu;
ESP32Encoder encoder1, encoder2;
Adafruit_VL53L0X sensor1, sensor2, sensor3;

// ================= TCA9548A =================
#define TCA9548A_ADDR 0x70
#define SCL_PIN 22
#define SDA_PIN 21

// ================= Motor =================
const int PWMA = 12, AIN1 = 33, AIN2 = 32;
const int PWMB = 14, BIN1 = 26, BIN2 = 27;
const int STBY = 25;
#define TIEN 1
#define LUI -1
#define DUNG 0

// ================= Encoder Pins =================
const int encoderPin1_1 = 19, encoderPin2_1 = 18;
const int encoderPin1_2 = 15, encoderPin2_2 = 2;

// ================= Global =================
std::stack<std::pair<int,int>> cellStack;
int speed = 160;

// ================= Maze & Robot Parameters =================
const int MAZE_SIZE = 16;
const int CELL_SIZE = 160;
const float WHEEL_DIAMETER = 34.0; // mm
// Dùng 1386 xung/vòng cho tính toán quay encoder
const float ENCODER_TICK_PER_REV = 1386.0;
const float WHEEL_BASE = 100.0;    // mm

float robotX = 0, robotY = 0, robotHeading = 0;
long initialEnc1 = 0, initialEnc2 = 0;
float initialHeadingValue = 0;

// ================= Maze Data =================
struct Cell {
  bool visited;
  int order;
  Cell() : visited(false), order(0) {}
};
Cell maze[MAZE_SIZE+5][MAZE_SIZE+5];
int cellOrder = 0;
int currentCellX = -1, currentCellY = -1;

// ================= Sensor Thresholds =================
const float FORWARD_THRESHOLD = 11.0;  // cm
const float LEFT_THRESHOLD = 14.0;     // cm
const float RIGHT_THRESHOLD = 18.0;    // cm
const int TURN_ANGLE = 90;

// ================= Bounce2 & Menu =================
#define BUTTON_SELECT 13
#define BUTTON_BACK 33
Bounce buttonSelect = Bounce();
Bounce buttonBack = Bounce();
volatile int encoderPos = 0;
volatile bool encoderMoved = false;
int currentItem = 0;
const int totalItems = 5;
const char* menuItems[totalItems] = {
  "Start DFS",
  "Hien thi dang so",
  "Hien thi dang cot",
  "Luu gia tri offset",
  "Reset cam bien"
};

bool inSubmenu = false;
bool displayMode = false;    // false: hiển thị số, true: hiển thị cột
bool sensorsActive = false;
bool dfsActive = false;

// ================= Hàm IRAM Encoder =================
void IRAM_ATTR readEncoder() {
  int stateA = digitalRead(ENCODER_PIN_A);
  int stateB = digitalRead(ENCODER_PIN_B);
  encoderPos += (stateA != stateB) ? 1 : -1;
  encoderMoved = true;
}

// ================= TCA Select for Bounce code =================
void tca9548a_selectChannel(uint8_t channel) {
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// ================= Menu Functions =================
void renderMenu() {
  display.clearDisplay();
  display.setTextSize(1);
  for (int i = 0; i < totalItems; i++) {
    if(i == currentItem)
      display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    else
      display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    display.setCursor(0, i*8);
    display.print(menuItems[i]);
  }
  display.display();
}

void numberDisplay() {
  display.clearDisplay();
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    tca9548a_selectChannel(sensorChannels[i]);
    uint16_t distance = sensors[i].readRangeContinuousMillimeters();
    int16_t distance_cm = distance / 10;
    if(sensors[i].timeoutOccurred()){
      Serial.print("Sensor "); Serial.print(i+1); Serial.println(" timeout");
      continue;
    }
    display.setCursor(0, i*8);
    display.print("S"); display.print(i+1); display.print(": ");
    display.print(distance_cm - offsets[i]);
    display.print(" cm");
  }
  display.display();
}

void handleEncoder() {
  if(encoderMoved) {
    currentItem = (encoderPos > 0) ? (currentItem + 1) % totalItems : (currentItem - 1 + totalItems) % totalItems;
    encoderPos = 0;
    encoderMoved = false;
    renderMenu();
  }
}

void saveOffset() {
  for (uint8_t i = 0; i < SENSOR_COUNT; i++){
    tca9548a_selectChannel(sensorChannels[i]);
    uint16_t distance = sensors[i].readRangeContinuousMillimeters();
    offsets[i] = distance / 10;
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0,0);
  display.print("Offset saved:");
  for(uint8_t i = 0; i < SENSOR_COUNT; i++){
    display.setCursor(0, i*8+10);
    display.print("S"); display.print(i+1); display.print(": ");
    display.print(offsets[i]); display.print(" cm");
  }
  display.display();
}

void handleButtons() {
  buttonSelect.update();
  buttonBack.update();
  if(buttonSelect.fell()){
    delay(200);
    if(!inSubmenu){
      inSubmenu = true;
      sensorsActive = true;
      if(currentItem == 0){
        dfsActive = true;
        // Khi chọn Start DFS, cập nhật encoder & heading ban đầu
        initialEnc1 = encoder1.getCount();
        initialEnc2 = encoder2.getCount();
        // Không dùng MPU để cân bằng, chỉ dùng encoder để quay
      } else if(currentItem == 1){
        displayMode = false;
      } else if(currentItem == 2){
        displayMode = true;
      } else if(currentItem == 3){
        sensorsActive = false;
        renderMenu();
        saveOffset();
      } else if(currentItem == 4){
        sensorsActive = false;
        renderMenu();
        // Có thể gọi resetSensors() nếu có
      }
    }
  }
  if(buttonBack.fell()){
    delay(200);
    if(inSubmenu){
      inSubmenu = false;
      sensorsActive = false;
      renderMenu();
    }
  }
  if(!inSubmenu){
    handleEncoder();
  }
  if(sensorsActive){
    numberDisplay();
  }
}

// ================= Encoder-based Turning Functions =================
// Sử dụng encoder để tính số xung cần quay
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

// ================= Update Position =================
void updateRobotPosition() {
  long curEnc1 = encoder1.getCount();
  long curEnc2 = encoder2.getCount();
  long dEnc1 = curEnc1 - initialEnc1;
  long dEnc2 = curEnc2 - initialEnc2;
  float d1 = (dEnc1 / 1386.0) * (PI * WHEEL_DIAMETER);
  float d2 = (dEnc2 / 1386.0) * (PI * WHEEL_DIAMETER);
  float dHeading = (d1 - d2) / WHEEL_BASE;
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
     Sensor 1 (channel 1): forward (> 11 cm)
     Sensor 2 (channel 2): left (> 14 cm)
     Sensor 3 (channel 4): right (> 18 cm)
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
  
  bool availForward = (dist_forward > FORWARD_THRESHOLD);
  bool availLeft = (dist_left > LEFT_THRESHOLD);
  bool availRight = (dist_right > RIGHT_THRESHOLD);
  
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
  
  tcaSelect(1); // Sensor 1: đo phía trước
  if(!sensor1.begin()) Serial.println("VL53L0X #1 failed!");
  
  tcaSelect(2); // Sensor 2: đo bên trái
  if(!sensor2.begin()) Serial.println("VL53L0X #2 failed!");
  
  tcaSelect(4); // Sensor 3: đo bên phải
  if(!sensor3.begin()) Serial.println("VL53L0X #3 failed!");
  
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
    Serial.print(" cm L: "); Serial.print(dist_left);
    Serial.print(" cm R: "); Serial.println(dist_right);
    
    dfsDecision(dist_forward, dist_left, dist_right);
    delay(50);
  }
}
