#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Encoder.h>
#include <Adafruit_VL53L0X.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

ESP32Encoder encoder1, encoder2;
Adafruit_VL53L0X sensor1, sensor2, sensor3;

#define TCA9548A_ADDR 0x70
#define SCL_PIN 22
#define SDA_PIN 21

const int PWMA = 12, AIN1 = 32, AIN2 = 33;
const int PWMB = 14, BIN1 = 26, BIN2 = 27;
const int STBY = 25;
int speed = 160;

#define TIEN -1
#define LUI 1
#define DUNG 0

const int encoderPin1_1 = 19, encoderPin2_1 = 18;
const int encoderPin1_2 = 15, encoderPin2_2 = 2;

float Kp = 1.5;
float Ki = 0.0;
float Kd = 0.5;

float error = 0, lastError = 0, integral = 0;

void tcaSelect(uint8_t channel) {
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

void Right_wheel(int control, int spd) {
  analogWrite(PWMA, spd);
  digitalWrite(AIN1, control == TIEN);
  digitalWrite(AIN2, control == LUI);
}

void Left_wheel(int control, int spd) {
  analogWrite(PWMB, spd);
  digitalWrite(BIN1, control == TIEN);
  digitalWrite(BIN2, control == LUI);
}

void stopMovement() {
  Right_wheel(DUNG, 0);
  Left_wheel(DUNG, 0);
}

void hienthiThongSo() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  display.print("Enc1: ");
  display.println(encoder1.getCount());
  display.print("Enc2: ");
  display.println(encoder2.getCount());

  tcaSelect(1);
  display.print("S1: ");
  display.println(sensor1.readRange());

  tcaSelect(2);
  display.print("S2: ");
  display.println(sensor2.readRange());

  tcaSelect(4);
  display.print("S3: ");
  display.println(sensor3.readRange());

  display.display();
  delay(50);
}

void di_thang_PID(long pulses) {
  encoder1.clearCount();
  encoder2.clearCount();
  integral = 0;
  lastError = 0;

  while (abs(encoder1.getCount()) < pulses && abs(encoder2.getCount()) < pulses) {
    tcaSelect(1);
    int distLeft = sensor1.readRange();

    tcaSelect(2);
    int distRight = sensor2.readRange();

    // Giữ đều mỗi bên 100mm
    error = (distLeft - 100) - (distRight - 100);  // = distLeft - distRight

    integral += error;
    float derivative = error - lastError;
    float correction = Kp * error + Ki * integral + Kd * derivative;
    lastError = error;

    int speedL = constrain(speed - correction, 0, 255);
    int speedR = constrain(speed + correction, 0, 255);

    Left_wheel(LUI, speedL);
    Right_wheel(LUI, speedR);

    hienthiThongSo();
  }

  stopMovement();
}

void re_trai() {
  encoder1.clearCount();
  encoder2.clearCount();
  while (abs(encoder1.getCount()) < 700 && abs(encoder2.getCount()) < 700) {
    Left_wheel(TIEN, speed * 1.03);
    Right_wheel(LUI, speed);
    hienthiThongSo();
  }
  stopMovement();
}

void re_phai() {
  encoder1.clearCount();
  encoder2.clearCount();
while (abs(encoder1.getCount()) < 700 && abs(encoder2.getCount()) < 700) {
    Left_wheel(LUI, speed * 1.03);
    Right_wheel(TIEN, speed);
    hienthiThongSo();
  }
  stopMovement();
}

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

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 failed!");
    while (1);
  }
  display.clearDisplay();

  encoder1.attachHalfQuad(encoderPin1_1, encoderPin2_1);
  encoder2.attachHalfQuad(encoderPin1_2, encoderPin2_2);

  tcaSelect(1); if (!sensor1.begin()) Serial.println("Sensor 1 fail");
  tcaSelect(2); if (!sensor2.begin()) Serial.println("Sensor 2 fail");
  tcaSelect(4); if (!sensor3.begin()) Serial.println("Sensor 3 fail");

  encoder1.clearCount();
  encoder2.clearCount();
}

void loop() {
  di_thang_PID(1800);  // đi 1 ô với cân bằng 100mm hai bên
  delay(300);
  re_trai();           // rẽ trái
  delay(300);
  di_thang_PID(1800);  // đi tiếp
  delay(300);
  re_phai();           // rẽ phải
  delay(300);
}