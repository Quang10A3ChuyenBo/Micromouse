#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Encoder.h>
#include <Adafruit_VL53L0X.h>
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
Adafruit_VL53L0X sensors[3];  // sensors[0]: forward, sensors[1]: left, sensors[2]: right
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

// ================ Global Parameters ================
int speed = 130;

// ================ Maze & Robot Parameters ================
const int MAZE_SIZE = 25;   // mm (điều chỉnh nếu cần)
const int CELL_SIZE = 165;    // mm (điều chỉnh nếu cần)
const float WHEEL_DIAMETER = 34.0;   // mm
const float ENCODER_TICK_PER_REV = 2400.0; // ticks/cell
const float WHEEL_BASE = 90;   // mm

float robotX = 0, robotY = 0;
// currentDirection: 0 = Bắc, 1 = Đông, 2 = Nam, 3 = Tây
int currentDirection = 0;
long initialEnc1 = 0, initialEnc2 = 0;
float initialHeadingValue = 0;

// ================ DFS Cell Storage ================
struct Cell {
  bool visited;
  int order;
  Cell() : visited(false), order(0) {}
};
Cell maze[MAZE_SIZE+5][MAZE_SIZE+5];
int cellOrder = 0;
int currentCellX = -1, currentCellY = -1;
std::stack<std::pair<int,int>> cellStack;

// ================ Sensor Thresholds (cm) ================
const float FORWARD_THRESHOLD = 11.0;
const float LEFT_THRESHOLD = 14.0;
const float RIGHT_THRESHOLD = 17.0;

// ================ TCA Channels for Sensors ================
const uint8_t sensorChannels[3] = {1, 2, 4};

// ================ Fixed Turning Constant ================
const float TURN_TICKS_90 = 550.0;  // ticks required for a 90° turn

// ================ TCA Select Function ================
void tcaSelect(uint8_t channel) {
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// ================ Dummy resetSensors() ================
void resetSensors() {
  // Code reset nếu cần; ở đây để trống.
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
  // Đi thẳng: bánh trái tiến, bánh phải lùi.
  Right_wheel(LUI, spd+5);
  Left_wheel(TIEN, spd);
}

void di_lui(int spd)
{
  Right_wheel(TIEN, spd+5);
  Left_wheel(LUI, spd);
}
// ================ Fixed Turning Functions ================
// re_phai: turn right using forward motion; re_trai: turn left using reverse motion.
// Dùng 660 ticks cho 90°; nếu turnFactor > 1, nhân thêm.
void re_phai(int spd, float turnFactor) {
  encoder1.setCount(0);
  encoder2.setCount(0);
  encoder1.clearCount();
  encoder2.clearCount();
  while (abs(encoder1.getCount()) < TURN_TICKS_90 || abs(encoder2.getCount()) < TURN_TICKS_90) {
      Right_wheel(TIEN, spd+5);
      Left_wheel(TIEN, spd);

      // display.setCursor(0, 0);
      // display.print("Enc1: "); display.print(encoder1.getCount());
      // display.print(" Enc2: "); display.println(encoder2.getCount());
      // display.display();

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

      // display.setCursor(0, 0);
      // display.print("Enc1: "); display.print(encoder1.getCount());
      // display.print(" Enc2: "); display.println(encoder2.getCount());
      // display.display();

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

      // display.setCursor(0, 0);
      // display.print("Enc1: "); display.print(encoder1.getCount());
      // display.print(" Enc2: "); display.println(encoder2.getCount());
      // display.display();

      delay(25);
    }
  
  stopMovement();
  currentDirection = (currentDirection + 2*(int)turnFactor) % 4;
}
// ================ Update Position ================
// Khi đi thẳng, cập nhật vị trí dựa trên trung bình encoder.
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

// ================ DFS Decision ================
// Lưu cell theo thứ tự; nếu không có hướng mới, backtracking dựa vào cellStack.
void dfsDecision(int dist_forward, int dist_left, int dist_right) {
  // Tính toán cell hiện tại (với offset 1200 mm)
  int cellX = (int)((robotX + 1200) / CELL_SIZE);
  int cellY = (int)((robotY + 1200) / CELL_SIZE);
  
  // Nếu robot bước vào cell mới, lưu cell cũ vào stack và cập nhật current cell
  if (cellX != currentCellX || cellY != currentCellY) {
    if (currentCellX != -1 && currentCellY != -1)
      cellStack.push({currentCellX, currentCellY});
    currentCellX = cellX;
    currentCellY = cellY;
    maze[cellX][cellY].visited = true;
    maze[cellX][cellY].order = ++cellOrder;
    
    // In ra OLED để debug: hiển thị cell và thứ tự
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
  bool canRight   = (dist_right > RIGHT_THRESHOLD);
  bool canLeft    = (dist_left > LEFT_THRESHOLD);
  
  if (canForward) {
    Serial.println("Decision: Move Forward");
    di_thang(speed);
  }
  else if (canRight) {
    Serial.println("Decision: Turn Right");
    stopMovement();
    delay(25);
    re_phai(90, 1);
  }
  else if (canLeft) {
    Serial.println("Decision: Turn Left");
    stopMovement();
    delay(25);
    re_trai(90, 1);
  }
  else {
    // Không có hướng khả thi, tiến hành backtracking dựa vào cell order.
    Serial.println("No available move; attempting backtracking...");
    bool foundCandidate = false;
    std::pair<int,int> candidate;
    while (!cellStack.empty()) {
      candidate = cellStack.top();
      cellStack.pop();
      // Chọn candidate nếu order của candidate nhỏ hơn (cũ hơn) cell hiện tại
      if (maze[candidate.first][candidate.second].order < maze[currentCellX][currentCellY].order) {
        foundCandidate = true;
        break;
      }
    }
    if (foundCandidate) {
      Serial.print("Backtracking to cell: ");
      Serial.print(candidate.first);
      Serial.print(", ");
      Serial.println(candidate.second);
      // Cập nhật current cell về candidate backtracked
      currentCellX = candidate.first;
      currentCellY = candidate.second;
      // Quay lại 180° rồi di chuyển lùi để rời khỏi cell cũ
      quay_lai(90, 1);  // Hàm quay lại 180° đã được định nghĩa
      di_lui(speed);     // Di chuyển lùi để đảm bảo rời khỏi cell hiện tại
    }
    else {
      Serial.println("No candidate for backtracking; moving forward.");
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
  
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("SSD1306 allocation failed!");
  }
  display.clearDisplay();
  
  encoder1.attachHalfQuad(encoderPin1_1, encoderPin2_1);
  encoder2.attachHalfQuad(encoderPin1_2, encoderPin2_2);
  
  tcaSelect(sensorChannels[0]); // Sensor 1: forward
  if (!sensors[0].begin()) Serial.println("VL53L0X #1 failed!");
  
  tcaSelect(sensorChannels[1]); // Sensor 2: left
  if (!sensors[1].begin()) Serial.println("VL53L0X #2 failed!");
  
  tcaSelect(sensorChannels[2]); // Sensor 3: right
  if (!sensors[2].begin()) display.println("VL53L0X #3 failed!");
  
  resetSensors();
  
  initialEnc1 = encoder1.getCount();
  initialEnc2 = encoder2.getCount();
  initialHeadingValue = 0;
  currentDirection = 0;
  
  display.clearDisplay();
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
  display.setTextSize(2); // Tăng kích thước chữ
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 10);  // Điều chỉnh vị trí bắt đầu vẽ
  display.print("Cell: ");
  display.print(cellX);
  display.print(",");
  display.println(cellY);
    display.setCursor(0, 35);  // Điều chỉnh vị trí bắt đầu vẽ cho dòng tiếp theo.
  display.print("Order: ");
  display.println(cellOrder);

  display.display();
  
  // Serial.print("F: "); Serial.print(dist_forward);
  // Serial.print(" cm L: "); Serial.print(dist_left);
  // Serial.print(" cm R: "); Serial.println(dist_right);
  
  dfsDecision(dist_forward, dist_left, dist_right);
  delay(50);
}