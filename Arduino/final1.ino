#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MPU6050.h>
#include <ESP32Encoder.h>
#include <Adafruit_VL53L0X.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
MPU6050 mpu;
ESP32Encoder encoder1, encoder2;
Adafruit_VL53L0X sensor1, sensor2, sensor3;

#define TCA9548A_ADDR 0x70
#define SCL_PIN 22
#define SDA_PIN 21

// Định nghĩa chân động cơ
const int PWMA = 12, AIN1 = 33, AIN2 = 32;  // Motor 1 (Trái)
const int PWMB = 14, BIN1 = 26, BIN2 = 27;  // Motor 2 (Phải)
const int STBY = 25;

#define TIEN 1
#define LUI -1
#define DUNG 0

// Chân encoder
const int encoderPin1_1 = 19, encoderPin2_1 = 18;
const int encoderPin1_2 = 15, encoderPin2_2 = 2;
int timequaylai=300;
float yaw = 0;
unsigned long lastTime = 0;
std::stack<std::pair<std::string, int> > s;
int speed = 160;

void tcaSelect(uint8_t channel) {
    Wire.beginTransmission(TCA9548A_ADDR);
    Wire.write(1 << channel);
    Wire.endTransmission();
}

// Điều khiển động cơ bên trái
void Left_wheel(int control, int speed) {
    analogWrite(PWMA, speed);
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

// Điều khiển động cơ bên phải
void Right_wheel(int control, int speed) {
    analogWrite(PWMB, speed);
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

float getYaw()
{
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0;
    lastTime = currentTime;

    float gZ = gz / 131.0;
    yaw = fmod(yaw + gZ * dt + 360, 360);
    return yaw;
}

void resetYaw()
{
  yaw = 0;
}

// Xe đi thẳng
void di_thang(int speed) {
    Right_wheel(TIEN, speed);
    Left_wheel(LUI, speed);
}
void turnRight(int speed, float angle, float start){
  resetYaw();
  float cay = getYaw();
  while (cay - start < angle)
  {
    Right_wheel(TIEN,speed);
    Left_wheel(TIEN,speed);
    cay = getYaw();
  }
  stopMovement();
}

// Hàm rẽ trái
void turnLeft(int speed, float angle, float start) {
  resetYaw();
  float cay = getYaw();
  while (start - cay  < angle)
  {
    Right_wheel(LUI,speed);
    Left_wheel(LUI,speed);
    cay = getYaw();
  }
  stopMovement();
 
 }



// Hàm quay đầu
// void quaydau(int speed) {
//   Right_wheel(LUI,speed);
//   Left_wheel(TIEN,speed);
//   delay(timequaylai);
//   stopMovement();
//}


// Dừng xe
void stopMovement() {
    Right_wheel(DUNG, 0);
    Left_wheel(DUNG, 0);
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
    
    digitalWrite(STBY, HIGH); // Bật chế độ Standby của TB6612

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
}

void loop() {
    

    VL53L0X_RangingMeasurementData_t measure1, measure2, measure3;

    tcaSelect(2);
    sensor1.rangingTest(&measure1, false);
    int dist1 = measure1.RangeMilliMeter / 10;

    tcaSelect(6);
    sensor2.rangingTest(&measure2, false);
    int dist2 = measure2.RangeMilliMeter / 10;

    tcaSelect(7);
    sensor3.rangingTest(&measure3, false);
    int dist3 = measure3.RangeMilliMeter / 10;

    long enc1 = encoder1.getCount();
    long enc2 = encoder2.getCount();

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print("Yaw (Z): "); display.println(yaw);
    display.print("Enc1: "); display.print(enc1);
    display.print(" Enc2: "); display.println(enc2);
    display.print("D1: "); display.print(dist1);
    display.print("cm D2: "); display.print(dist2);
    display.print("cm D3: "); display.print(dist3);
    display.println("cm");
    display.display();

    Serial.print("Yaw: "); Serial.print(yaw);
    Serial.print(" Enc1: "); Serial.print(enc1);
    Serial.print(" Enc2: "); Serial.print(enc2);
    Serial.print(" D1: "); Serial.print(dist1);
    Serial.print("cm D2: "); Serial.print(dist2);
    Serial.print("cm D3: "); Serial.print(dist3);
    Serial.println("cm");

    // Xe đi thẳng
    if ( dist1 > 30) s.push(std::make_pair("re_phai", 0));
    if ( dist3 > 30) s.push(std::make_pair("re_trai", 0));
    if ( dist2 > 30) s.push(std::make_pair("thang", 1));
    if ( dist2 <= 30 && dist1 <= 30 && dist3 <= 30)
    {
       if (s.empty()) {
            stopMovement();
            return;
        }
        cpoint = 0;
       while(!s.empty()){
       std::pair<std::string, int> ac = s.top();
       s.pop();
       if ( ac.first() == "thang") di_thang(speed);
       start = getYaw();
       if (ac.first() == "re_phai" && ac.second() == 1) { turnLeft(speed, 90, start); cpoint = 0;}
       else { turnLeft(speed, 90, start); cpoint = 1;}
       
       if (ac.first() == "re_trai" && ac.second() == 1) { turnRight(speed, 90, start); cpoint = 0;}
       else { turnRight(speed, 90, start); cpoint = 1;}
       }
       if (cpoint == 1) break;
    }
    else
    {
      if (dist2 > 30) di_thang(speed);
      else
      {
        std::pair<std::string, int> ac = s.top();
        if ( ac.fisrt() == "di_thang") s.pop();
        if ( ac.first() == "re_phai" && dist1 > 30) turnRight(speed, 90, start);
        if ( ac.first() == "re_trai" && dist3 > 30) turnLeft(speed, 90, start);
        s.pop();
        ac.second() = 1;
        s.push(ac);
      }
    }
    delay(50);
}