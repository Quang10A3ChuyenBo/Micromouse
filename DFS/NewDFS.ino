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

// ================ Global Variables ================
int speed = 160;

// ================ Maze & Robot Parameters ================
const int MAZE_SIZE = 3000;
const int CELL_SIZE = 165;
const float WHEEL_DIAMETER = 34.0;   // mm
// 1 ô di chuyển = 2450 xung
const float ENCODER_TICK_PER_REV = 2450.0;
const float WHEEL_BASE = 90;      // mm

float robotX = 0, robotY = 0;
// Quản lý hướng bằng biến currentDirection: 0 = Bắc, 1 = Đông, 2 = Nam, 3 = Tây.
int currentDirection = 0;
long initialEnc1 = 0, initialEnc2 = 0;
float initialHeadingValue = 0;  // = 0

// ================ Sensor Thresholds (cm) ================
const float FORWARD_THRESHOLD = 11.0;
const float LEFT_THRESHOLD = 14.0;
const float RIGHT_THRESHOLD = 18.0;

// ================ Cài đặt cảm biến (TCA channels) ================
const uint8_t sensorChannels[3] = {1, 2, 4};

const float TURN_TICKS_90 = 650.0;

// ================ IRAM Encoder ISR ================
void IRAM_ATTR readEncoder() {
  int stateA = digitalRead(ENCODER_PIN_A);
  int stateB = digitalRead(ENCODER_PIN_B);
  // Giả sử thư viện encoder tự cập nhật, ISR ở đây không cần cập nhật lại encoder1, encoder2.
  // (Encoder library thường tự động cập nhật giá trị.)
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
  // Đi thẳng: bánh trái tiến, bánh phải lùi.
  Right_wheel(LUI, spd);
  Left_wheel(TIEN, spd);
}

// ================ Fixed Turning Functions ================
// Mỗi 90° quay = 600 xung.
// Rẽ phải: xe tiến (cả 2 bánh chạy tiến, bánh trái nhanh hơn bánh phải).
void turnRightFixed(int spd, float turnFactor) {
  long startL = encoder1.getCount();
  long startR = encoder2.getCount();
  float req = TURN_TICKS_90 * turnFactor;
  while (true) {
    long diffL = encoder1.getCount() - startL;
    long diffR = encoder2.getCount() - startR;
    // Khi đi tiến, cả 2 bánh tăng số xung, để quay phải cần bánh trái tăng nhanh hơn bánh phải.
    if ((diffL - diffR) >= req) break;
    Left_wheel(TIEN, spd-30);
    Right_wheel(TIEN, spd - 30);
  }
  stopMovement();
  currentDirection = (currentDirection + (int)turnFactor) % 4;
}

// Rẽ trái: xe lùi (cả 2 bánh chạy lùi, bánh phải chạy lùi nhanh hơn bánh trái).
void turnLeftFixed(int spd, float turnFactor) {
  long startL = encoder1.getCount();
  long startR = encoder2.getCount();
  float req = TURN_TICKS_90 * turnFactor;
  while (true) {
    long diffL = encoder1.getCount() - startL;  // Khi lùi, giá trị giảm
    long diffR = encoder2.getCount() - startR;
    // Ta tính giá trị âm: muốn |diffR| > |diffL| với hiệu số đạt req.
    if (((-diffR) - (-diffL)) >= req) break;
    Right_wheel(TIEN, spd-30);
    Left_wheel(TIEN, spd - 30);
  }
  stopMovement();
  currentDirection = (currentDirection + 3) % 4;
}

// ================ Update Position ================
// Khi đi thẳng, cập nhật vị trí dựa trên currentDirection.
void updateRobotPosition() {
  long curE1 = encoder1.getCount();
  long curE2 = encoder2.getCount();
  long dE = (curE1 + curE2) / 2 - (initialEnc1 + initialEnc2) / 2;
  float dist = (dE / ENCODER_TICK_PER_REV) * (PI * WHEEL_DIAMETER);
  if (currentDirection == 0) robotY += dist;
  else if (currentDirection == 1) robotX += dist;
  else if (currentDirection == 2) robotY -= dist;
  else if (currentDirection == 3) robotX -= dist;
  initialEnc1 = curE1;
  initialEnc2 = curE2;
}

// ================ DFS Decision ================
// Đơn giản: nếu cảm biến phía trước đủ, đi thẳng; nếu không, nếu bên phải đủ, quay phải; nếu không, nếu bên trái đủ, quay trái; còn lại, đi thẳng.
void dfsDecision(int dist_forward, int dist_left, int dist_right) {
  if (dist_forward > FORWARD_THRESHOLD) {
    Serial.println("Decision: Move Forward");
    di_thang(speed);
  } else if (dist_right > RIGHT_THRESHOLD) {
    Serial.println("Decision: Turn Right");
    turnRightFixed(speed, 1);
    di_thang(speed);
  } else if (dist_left > LEFT_THRESHOLD) {
    Serial.println("Decision: Turn Left");
    turnLeftFixed(speed, 1);
    di_thang(speed);
  } else {
    Serial.println("No available move; moving forward.");
    di_thang(speed);
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
  
  tcaSelect(1); // Sensor 1: forward
  if (!sensors[0].begin()) Serial.println("VL53L0X #1 failed!");
  
  tcaSelect(2); // Sensor 2: left
  if (!sensors[1].begin()) Serial.println("VL53L0X #2 failed!");
  
  tcaSelect(4); // Sensor 3: right
  if (!sensors[2].begin()) Serial.println("VL53L0X #3 failed!");
  
  // Khởi động xe chạy luôn
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
  
  // Hiển thị thông tin (có thể xóa nếu không cần)
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
  delay(40);
}
