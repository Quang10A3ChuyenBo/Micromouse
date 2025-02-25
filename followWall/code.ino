#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Encoder.h>
#include <Adafruit_VL53L0X.h>
#include <math.h>

// OLED & I2C
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDR 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Encoder & VL53L0X
ESP32Encoder encoder1, encoder2;
Adafruit_VL53L0X sensors[3];  // sensors[0]: forward, sensors[1]: left, sensors[2]: right

// TCA9548A
#define TCA9548A_ADDR 0x70
#define SCL_PIN 22
#define SDA_PIN 21

// Motor Pins
const int PWMA = 12, AIN1 = 33, AIN2 = 32;
const int PWMB = 14, BIN1 = 26, BIN2 = 27;
const int STBY = 25;
#define TIEN 1
#define LUI -1
#define DUNG 0

// Encoder Pins
#define ENCODER_PIN_A 19
#define ENCODER_PIN_B 18
const int encoderPin1_1 = 19, encoderPin2_1 = 18;
const int encoderPin1_2 = 15, encoderPin2_2 = 2;

// Global Parameters
int speed = 160;

// Sensor thresholds (cm)
const float FORWARD_THRESHOLD = 11.0;
const float LEFT_THRESHOLD = 14.0;
const float RIGHT_THRESHOLD = 17.0;

// TCA channels for sensors: Sensor 1 (forward): 1, Sensor 2 (left): 2, Sensor 3 (right): 4.
const uint8_t sensorChannels[3] = {1, 2, 4};

// Fixed turning constant: số tick cần cho 90° quay (có thể điều chỉnh)
const float TURN_TICKS_90 = 660.0;

// TCA Select Function
void tcaSelect(uint8_t channel) {
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// Dummy resetSensors (nếu cần)
void resetSensors() { }

// Motor Control Functions
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

// Fixed Turning Functions (sử dụng encoder)
// re_phai: quay phải – xe chạy tiến, bánh trái chạy nhanh hơn (speed không trừ 30 trên bánh trái)
void re_phai(int spd, float turnFactor) {
  encoder1.setCount(0);
  encoder2.setCount(0);
  // Vòng lặp cho đến khi đạt yêu cầu tick
  float req = TURN_TICKS_90 * turnFactor;
  while ( (abs(encoder1.getCount()) < req) || (abs(encoder2.getCount()) < req) ) {
    Right_wheel(TIEN, spd);
    Left_wheel(TIEN, spd - 30);
    delayMicroseconds(100);
  }
  stopMovement();
}
// re_trai: quay trái – xe chạy lùi, bánh phải chạy nhanh hơn (speed không trừ 30 trên bánh phải)
void re_trai(int spd, float turnFactor) {
  encoder1.setCount(0);
  encoder2.setCount(0);
  float req = TURN_TICKS_90 * turnFactor;
  while ( (abs(encoder1.getCount()) < req) || (abs(encoder2.getCount()) < req) ) {
    Right_wheel(LUI, spd);
    Left_wheel(LUI, spd - 30);
    delayMicroseconds(100);
  }
  stopMovement();
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
  if (!sensors[2].begin()) Serial.println("VL53L0X #3 failed!");
  
  resetSensors();
  
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
  
  // Simple wall-following: ưu tiên đi thẳng > rẽ phải > rẽ trái.
  if (dist_forward > FORWARD_THRESHOLD) {
    Serial.println("Move Forward");
    di_thang(speed);
  } else if (dist_right > RIGHT_THRESHOLD) {
    Serial.println("Turn Right");
    stopMovement();
    re_phai(90, 1);
  } else if (dist_left > LEFT_THRESHOLD) {
    Serial.println("Turn Left");
    stopMovement();
    re_trai(90, 1);
  } else {
    Serial.println("No available move; moving forward.");
    di_thang(speed);
  }
  
  delay(40);
}
