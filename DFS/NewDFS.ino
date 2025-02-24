#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Encoder.h>
#include <Adafruit_VL53L0X.h>
#include <Bounce2.h>
#include <stack>
#include <math.h>

// ================ OLED & I2C ================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDR 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ================ Encoder & VL53L0X ================
ESP32Encoder encoder1, encoder2;
Adafruit_VL53L0X sensors[3];  // 0: forward, 1: left, 2: right
int16_t offsets[3] = {0, 0, 0};

// ================ TCA9548A ================
#define TCA9548A_ADDR 0x70
#define SCL_PIN 22
#define SDA_PIN 21

// ================ Motor Pins ================
const int PWMA = 12, AIN1 = 33, AIN2 = 32;
const int PWMB = 14, BIN1 = 26, BIN2 = 27;
const int STBY = 25;
#define TIEN 1
#define LUI -1
#define DUNG 0

// ================ Encoder Pins ================
#define ENCODER_PIN_A 19
#define ENCODER_PIN_B 18
const int encoderPin1_1 = 19, encoderPin2_1 = 18;
const int encoderPin1_2 = 15, encoderPin2_2 = 2;

// ================ Global Variables ================
std::stack<std::pair<int,int>> cellStack;
int speed = 160;

// ================ Maze & Robot Parameters ================
const int MAZE_SIZE = 16;
const int CELL_SIZE = 160;
const float WHEEL_DIAMETER = 34.0;   // mm
// 1 ô di chuyển = 2450 xung
const float ENCODER_TICK_PER_REV = 2450.0;
const float WHEEL_BASE = 100.0;      // mm

float robotX = 0, robotY = 0;
// Hướng của xe được quản lý qua currentDirection (0: Bắc, 1: Đông, 2: Nam, 3: Tây)
int currentDirection = 0;
long initialEnc1 = 0, initialEnc2 = 0;
float initialHeadingValue = 0;  // Đặt = 0

struct Cell {
  bool visited;
  int order;
  int floodFillValue;
  Cell() : visited(false), order(0), floodFillValue(9999) {}
};
Cell maze[MAZE_SIZE+5][MAZE_SIZE+5];
int cellOrder = 0;
int currentCellX = -1, currentCellY = -1;

// ================ Sensor Thresholds (cm) ================
const float FORWARD_THRESHOLD = 11.0;
const float LEFT_THRESHOLD = 14.0;
const float RIGHT_THRESHOLD = 18.0;
const int TURN_ANGLE = 90;

// ================ Bounce2 & Menu ================
#define BUTTON_SELECT 13
#define BUTTON_BACK 33
Bounce buttonSelect = Bounce();
Bounce buttonBack = Bounce();
volatile int encoderPos = 0;
volatile bool encoderMoved = false;
int currentItem = 0;
const int totalItems = 5;
const char* menuItems[5] = {
  "Start DFS", "Hien thi dang so", "Hien thi dang cot", "Luu gia tri offset", "Reset cam bien"
};

bool inSubmenu = false;
bool displayMode = false;  // false: số, true: cột
bool sensorsActive = false;
bool dfsActive = false;

// Cài đặt kênh cảm biến: Sensor 1: 1, Sensor 2: 2, Sensor 3: 4
const uint8_t sensorChannels[3] = {1, 2, 4};

// ================ IRAM Encoder ISR ================
void IRAM_ATTR readEncoder() {
  int stateA = digitalRead(ENCODER_PIN_A);
  int stateB = digitalRead(ENCODER_PIN_B);
  encoderPos += (stateA != stateB) ? 1 : -1;
  encoderMoved = true;
}

// ================ TCA Select Function ================
void tcaSelect(uint8_t channel) {
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// ================ Dummy resetSensors() ================
void resetSensors() {
  // Nếu cần, thêm code reset cho cảm biến. Ở đây để trống.
}

// ================ Motor Control Functions ================
void Left_wheel(int control, int spd) {
  analogWrite(PWMA, spd);
  digitalWrite(AIN1, (control == TIEN));
  digitalWrite(AIN2, (control == LUI));
}

void Right_wheel(int control, int spd) {
  analogWrite(PWMB, spd);
  digitalWrite(BIN1, (control == TIEN));
  digitalWrite(BIN2, (control == LUI));
}

void stopMovement() {
  Right_wheel(DUNG, 0);
  Left_wheel(DUNG, 0);
}

void di_thang(int spd) {
  // Giả sử: để đi thẳng, bánh trái tiến, bánh phải lùi.
  Right_wheel(LUI, spd);
  Left_wheel(TIEN, spd);
}

// ================ Menu Functions ================
void renderMenu() {
  display.clearDisplay();
  display.setTextSize(1);
  for (int i = 0; i < totalItems; i++) {
    if (i == currentItem)
      display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    else
      display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    display.setCursor(0, i * 8);
    display.print(menuItems[i]);
  }
  display.display();
}

void numberDisplay() {
  display.clearDisplay();
  for (uint8_t i = 0; i < 3; i++) {
    tcaSelect(sensorChannels[i]);
    VL53L0X_RangingMeasurementData_t measure;
    sensors[i].rangingTest(&measure, false);
    int distance_cm = measure.RangeMilliMeter / 10;
    display.setCursor(0, i * 8);
    display.print("S"); display.print(i+1); display.print(": ");
    display.print(distance_cm - offsets[i]);
    display.print(" cm");
  }
  display.display();
}

void handleEncoder() {
  if (encoderMoved) {
    currentItem = (encoderPos > 0) ? (currentItem + 1) % totalItems : (currentItem - 1 + totalItems) % totalItems;
    encoderPos = 0;
    encoderMoved = false;
    renderMenu();
  }
}

void saveOffset() {
  for (uint8_t i = 0; i < 3; i++) {
    tcaSelect(sensorChannels[i]);
    VL53L0X_RangingMeasurementData_t measure;
    sensors[i].rangingTest(&measure, false);
    offsets[i] = measure.RangeMilliMeter / 10;
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("Offset saved:");
  for (uint8_t i = 0; i < 3; i++) {
    display.setCursor(0, i * 8 + 10);
    display.print("S"); display.print(i+1); display.print(": ");
    display.print(offsets[i]); display.print(" cm");
  }
  display.display();
}

void handleButtons() {
  buttonSelect.update();
  buttonBack.update();
  if (buttonSelect.fell()) {
    delay(200);
    if (!inSubmenu) {
      inSubmenu = true;
      sensorsActive = true;
      if (currentItem == 0) {
        dfsActive = true; // Start DFS
        initialEnc1 = encoder1.getCount();
        initialEnc2 = encoder2.getCount();
        initialHeadingValue = 0;
        currentDirection = 0; // Khởi tạo hướng: Bắc
      } else if (currentItem == 1) {
        displayMode = false;
      } else if (currentItem == 2) {
        displayMode = true;
      } else if (currentItem == 3) {
        sensorsActive = false;
        renderMenu();
        saveOffset();
      } else if (currentItem == 4) {
        sensorsActive = false;
        renderMenu();
        // resetSensors() nếu cần
      }
    }
  }
  if (buttonBack.fell()) {
    delay(200);
    if (inSubmenu) {
      inSubmenu = false;
      sensorsActive = false;
      renderMenu();
    }
  }
  if (!inSubmenu) handleEncoder();
  if (sensorsActive) numberDisplay();
}

// ================ Fixed Turning Functions ================
// Mỗi 90° quay = 600 xung.
// Khi quay phải: bánh trái tăng 600, bánh phải giảm 600.
// Khi quay trái: bánh trái giảm 600, bánh phải tăng 600.
void turnRightFixed(int spd, float turnFactor) {
  long startL = encoder1.getCount();
  long startR = encoder2.getCount();
  float req = 600.0 * turnFactor;
  while (true) {
    long diffL = encoder1.getCount() - startL;   // Bánh trái tiến (tăng)
    long diffR = encoder2.getCount() - startR;   // Bánh phải lùi (âm)
    if (diffL >= req && diffR <= -req) break;
    Right_wheel(LUI, spd-60);
    Left_wheel(TIEN, spd-60);
  }
  stopMovement();
  currentDirection = (currentDirection + (int)turnFactor) % 4;
}

void turnLeftFixed(int spd, float turnFactor) {
  long startL = encoder1.getCount();
  long startR = encoder2.getCount();
  float req = 600.0 * turnFactor;
  while (true) {
    long diffL = encoder1.getCount() - startL;   // Bánh trái lùi (âm)
    long diffR = encoder2.getCount() - startR;   // Bánh phải tiến
    if (diffL <= -req && diffR >= req) break;
    Right_wheel(TIEN, spd-60);
    Left_wheel(LUI, spd-60);
  }
  stopMovement();
  currentDirection = (currentDirection - (int)turnFactor + 4) % 4;
}

// ================ Update Position ================
// Khi đi thẳng, cập nhật vị trí dựa trên currentDirection.
void updateRobotPosition() {
  long curE1 = encoder1.getCount();
  long curE2 = encoder2.getCount();
  long dE1 = curE1 - initialEnc1;
  long dE2 = curE2 - initialEnc2;
  float avg = (dE1 + dE2) / 2.0;
  float dist = (avg / ENCODER_TICK_PER_REV) * (PI * WHEEL_DIAMETER);
  if (currentDirection == 0) robotY += dist;
  else if (currentDirection == 1) robotX += dist;
  else if (currentDirection == 2) robotY -= dist;
  else if (currentDirection == 3) robotX -= dist;
  initialEnc1 = curE1;
  initialEnc2 = curE2;
}

// ================ isValidCell ================
bool isValidCell(int x, int y) {
  return (x >= 0 && x < MAZE_SIZE && y >= 0 && y < MAZE_SIZE);
}

// ================ DFS Decision ================
// Dựa trên currentDirection (0: Bắc, 1: Đông, 2: Nam, 3: Tây)
void dfsDecision(int dist_forward, int dist_left, int dist_right) {
  int cellX = (int)((robotX + 1200) / CELL_SIZE);
  int cellY = (int)((robotY + 1200) / CELL_SIZE);
  
  if (cellX != currentCellX || cellY != currentCellY) {
    if (currentCellX != -1 && currentCellY != -1)
      cellStack.push({currentCellX, currentCellY});
    currentCellX = cellX;
    currentCellY = cellY;
    maze[cellX][cellY].visited = true;
    maze[cellX][cellY].order = ++cellOrder;
    Serial.print("Entered cell: ");
    Serial.print(cellX);
    Serial.print(", ");
    Serial.println(cellY);
  }
  
  bool availForward = (dist_forward > FORWARD_THRESHOLD);
  bool availLeft = (dist_left > LEFT_THRESHOLD);
  bool availRight = (dist_right > RIGHT_THRESHOLD);
  
  // Xác định ô phía trước dựa trên currentDirection
  int nx, ny;
  if (currentDirection == 0) { nx = currentCellX; ny = currentCellY + 1; }
  else if (currentDirection == 1) { nx = currentCellX + 1; ny = currentCellY; }
  else if (currentDirection == 2) { nx = currentCellX; ny = currentCellY - 1; }
  else { nx = currentCellX - 1; ny = currentCellY; }
  bool forwardUnvisited = isValidCell(nx, ny) && !maze[nx][ny].visited;
  
  // Ô bên phải: (currentDirection + 1) mod 4
  int rx, ry;
  int rDir = (currentDirection + 1) % 4;
  if (rDir == 0) { rx = currentCellX; ry = currentCellY + 1; }
  else if (rDir == 1) { rx = currentCellX + 1; ry = currentCellY; }
  else if (rDir == 2) { rx = currentCellX; ry = currentCellY - 1; }
  else { rx = currentCellX - 1; ry = currentCellY; }
  bool rightUnvisited = isValidCell(rx, ry) && !maze[rx][ry].visited;
  
  // Ô bên trái: (currentDirection + 3) mod 4
  int lx, ly;
  int lDir = (currentDirection + 3) % 4;
  if (lDir == 0) { lx = currentCellX; ly = currentCellY + 1; }
  else if (lDir == 1) { lx = currentCellX + 1; ly = currentCellY; }
  else if (lDir == 2) { lx = currentCellX; ly = currentCellY - 1; }
  else { lx = currentCellX - 1; ly = currentCellY; }
  bool leftUnvisited = isValidCell(lx, ly) && !maze[lx][ly].visited;
  
  if (availForward && forwardUnvisited) {
    Serial.println("Decision: Move Forward");
    di_thang(speed);
  } else if (availRight && rightUnvisited) {
    Serial.println("Decision: Turn Right");
    turnRightFixed(speed, 1);
    di_thang(speed);
  } else if (availLeft && leftUnvisited) {
    Serial.println("Decision: Turn Left");
    turnLeftFixed(speed, 1);
    di_thang(speed);
  } else {
    if (!cellStack.empty()) {
      auto prevCell = cellStack.top(); cellStack.pop();
      Serial.print("Backtracking to cell: ");
      Serial.print(prevCell.first);
      Serial.print(", ");
      Serial.println(prevCell.second);
      int dx = prevCell.first - currentCellX;
      int dy = prevCell.second - currentCellY;
      int desired;
      if (dx > 0) desired = 1;
      else if (dx < 0) desired = 3;
      else if (dy > 0) desired = 0;
      else desired = 2;
      while (currentDirection != desired) {
        turnRightFixed(speed, 1);
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
  
  // Không dùng MPU
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("SSD1306 allocation failed!");
  }
  display.clearDisplay();
  
  encoder1.attachHalfQuad(encoderPin1_1, encoderPin2_1);
  encoder2.attachHalfQuad(encoderPin1_2, encoderPin2_2);
  
  tcaSelect(1); // Sensor 1: forward
  if (!sensors[0].begin()) Serial.println("VL53L0X #1 failed!");
  
  tcaSelect(2); // Sensor 2: left
  if (!sensors[1].begin()) Serial.println("VL53L0X #2 failed!");
  
  tcaSelect(4); // Sensor 3: right
  if (!sensors[2].begin()) Serial.println("VL53L0X #3 failed!");
  
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
  
  initialEnc1 = encoder1.getCount();
  initialEnc2 = encoder2.getCount();
  initialHeadingValue = 0;
  currentDirection = 0;
}

void loop() {
  if (!dfsActive) {
    handleEncoder();
    numberDisplay();
    handleButtons();
  } else {
    VL53L0X_RangingMeasurementData_t m0, m1, m2;
    tcaSelect(sensorChannels[0]); // Sensor 1: forward
    delay(10);
    sensors[0].rangingTest(&m0, false);
    int dist_forward = m0.RangeMilliMeter / 10;
    
    tcaSelect(sensorChannels[1]); // Sensor 2: left
    delay(10);
    sensors[1].rangingTest(&m1, false);
    int dist_left = m1.RangeMilliMeter / 10;
    
    tcaSelect(sensorChannels[2]); // Sensor 3: right
    delay(10);
    sensors[2].rangingTest(&m2, false);
    int dist_right = m2.RangeMilliMeter / 10;
    
    updateRobotPosition();
    
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print("Enc: "); display.println(encoder1.getCount());
    display.print("F: "); display.print(dist_forward); display.println(" cm");
    display.print("L: "); display.print(dist_left); display.println(" cm");
    display.print("R: "); display.print(dist_right); display.println(" cm");
    display.display();
    
    Serial.print("F: "); Serial.print(dist_forward);
    Serial.print(" cm L: "); Serial.print(dist_left);
    Serial.print(" cm R: "); Serial.println(dist_right);
    
    dfsDecision(dist_forward, dist_left, dist_right);
    delay(50);
  }
}
