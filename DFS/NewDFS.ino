#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Encoder.h>
#include <Adafruit_VL53L0X.h>
#include <stack>
#include <math.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDR 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

ESP32Encoder encoder1, encoder2;
Adafruit_VL53L0X sensors[3];
int16_t offsets[3] = {0, 0, 0};

#define TCA9548A_ADDR 0x70
#define SCL_PIN 22
#define SDA_PIN 21

const int PWMA = 12, AIN1 = 33, AIN2 = 32;
const int PWMB = 14, BIN1 = 26, BIN2 = 27;
const int STBY = 25;
#define TIEN 1
#define LUI -1
#define DUNG 0

#define ENCODER_PIN_A 19
#define ENCODER_PIN_B 18
const int encoderPin1_1 = 19, encoderPin2_1 = 18;
const int encoderPin1_2 = 15, encoderPin2_2 = 2;

int speed = 130;

const int MAZE_SIZE = 25;
const int CELL_SIZE = 165;
const float WHEEL_DIAMETER = 34.0;
const float ENCODER_TICK_PER_REV = 2400.0;
const float WHEEL_BASE = 90;

float robotX = 0, robotY = 0;
int currentDirection = 0;
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
std::stack<std::pair<int,int>> cellStack;

const float FORWARD_THRESHOLD = 11.0;
const float LEFT_THRESHOLD = 14.0;
const float RIGHT_THRESHOLD = 17.0;

const uint8_t sensorChannels[3] = {1, 2, 4};
const float TURN_TICKS_90 = 550.0;

void tcaSelect(uint8_t channel) {
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

void resetSensors() {}

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
  Right_wheel(LUI, spd+5);
  Left_wheel(TIEN, spd);
}
void di_lui(int spd) {
  Right_wheel(TIEN, spd+5);
  Left_wheel(LUI, spd);
}
void re_phai(int spd, float turnFactor) {
  encoder1.setCount(0);
  encoder2.setCount(0);
  encoder1.clearCount();
  encoder2.clearCount();
  while (abs(encoder1.getCount()) < TURN_TICKS_90 || abs(encoder2.getCount()) < TURN_TICKS_90) {
    Right_wheel(TIEN, spd+5);
    Left_wheel(TIEN, spd);
    delay(25);
  }
  stopMovement();
  currentDirection = (currentDirection + (int)turnFactor) % 4;
}
void re_trai(int spd, float turnFactor) {
  encoder1.setCount(0);
  encoder2.setCount(0);
  encoder1.clearCount();
  encoder2.clearCount();
  while (abs(encoder1.getCount()) < TURN_TICKS_90 || abs(encoder2.getCount()) < TURN_TICKS_90 ) {
    Right_wheel(LUI, spd+5);
    Left_wheel(LUI, spd);
    delay(25);
  }
  stopMovement();
  currentDirection = (currentDirection + 3) % 4;
}
void quay_lai(int spd, float turnFactor) {
  encoder1.setCount(0);
  encoder2.setCount(0);
  encoder1.clearCount();
  encoder2.clearCount();
  while (abs(encoder1.getCount()) < 1500 || abs(encoder2.getCount()) < 1500) {
    Right_wheel(TIEN, spd+5);
    Left_wheel(TIEN, spd);
    delay(25);
  }
  stopMovement();
  currentDirection = (currentDirection + 2*(int)turnFactor) % 4;
}
void updateRobotPosition() {
  long curE1 = encoder1.getCount();
  long curE2 = encoder2.getCount();
  long dE = ((curE1 + curE2) / 2) - ((initialEnc1 + initialEnc2) / 2);
  float dist = (dE / ENCODER_TICK_PER_REV) * (PI * WHEEL_DIAMETER);
  if (currentDirection == 0) robotY += dist;
  else if (currentDirection == 1) robotX += dist;
  else if (currentDirection == 2) robotY -= dist;
  else if (currentDirection == 3) robotX -= dist;
  initialEnc1 = curE1;
  initialEnc2 = curE2;
}
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
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print("Cell: ");
    display.print(cellX);
    display.print(",");
    display.println(cellY);
    display.print("Order: ");
    display.println(cellOrder);
    display.display();
    delay(50);
  }
  bool canForward = (dist_forward > FORWARD_THRESHOLD);
  bool canRight = (dist_right > RIGHT_THRESHOLD);
  bool canLeft = (dist_left > LEFT_THRESHOLD);
  if (canForward) {
    di_thang(speed);
  }
  else if (canRight) {
    stopMovement();
    delay(25);
    re_phai(90, 1);
  }
  else if (canLeft) {
    stopMovement();
    delay(25);
    re_trai(90, 1);
  }
  else {
    bool foundCandidate = false;
    std::pair<int,int> candidate;
    while (!cellStack.empty()) {
      candidate = cellStack.top();
      cellStack.pop();
      if (maze[candidate.first][candidate.second].order < maze[currentCellX][currentCellY].order) {
        foundCandidate = true;
        break;
      }
    }
    if (foundCandidate) {
      currentCellX = candidate.first;
      currentCellY = candidate.second;
      quay_lai(90, 1);
      di_lui(speed);
    }
    else {
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
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.clearDisplay();
  encoder1.attachHalfQuad(encoderPin1_1, encoderPin2_1);
  encoder2.attachHalfQuad(encoderPin1_2, encoderPin2_2);
  tcaSelect(sensorChannels[0]);
  sensors[0].begin();
  tcaSelect(sensorChannels[1]);
  sensors[1].begin();
  tcaSelect(sensorChannels[2]);
  sensors[2].begin();
  resetSensors();
  initialEnc1 = encoder1.getCount();
  initialEnc2 = encoder2.getCount();
  initialHeadingValue = 0;
  currentDirection = 0;
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.println("Khoi dong...");
  display.display();
  delay(50);
}

void loop() {
  VL53L0X_RangingMeasurementData_t m0, m1, m2;
  tcaSelect(sensorChannels[0]);
  delay(25);
  sensors[0].rangingTest(&m0, false);
  int dist_forward = m0.RangeMilliMeter / 10;
  tcaSelect(sensorChannels[1]);
  delay(25);
  sensors[1].rangingTest(&m1, false);
  int dist_left = m1.RangeMilliMeter / 10;
  tcaSelect(sensorChannels[2]);
  delay(25);
  sensors[2].rangingTest(&m2, false);
  int dist_right = m2.RangeMilliMeter / 10;
  updateRobotPosition();
  int cellX = (int)((robotX + 1200) / CELL_SIZE);
  int cellY = (int)((robotY + 1200) / CELL_SIZE);
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 10);
  display.print("Cell: ");
  display.print(cellX);
  display.print(",");
  display.println(cellY);
  display.setCursor(0, 35);
  display.print("Order: ");
  display.println(cellOrder);
  display.display();
  dfsDecision(dist_forward, dist_left, dist_right);
  delay(50);
}
