#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>         mm   
#include <MPU6050.h>
#include <ESP32Encoder.h>
#include <Adafruit_VL53L0X.h>
#include <stack>
#include <math.h>
#include <bits/stdc++.h>

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

float yaw = 0;
unsigned long lastTime = 0;

std::stack<std::pair<int,int> > cellStack;
int speed = 160;

const int BUTTON1_PIN = 17;
const int BUTTON2_PIN = 34;
// Enums for Modes
enum RobotMode {
  IDLE,
  MAPPING
};
RobotMode currentMode = IDLE;

const int MAZE_SIZE = 16;          
const int CELL_SIZE = 160;        
const float WHEEL_DIAMETER = 60.0;   
const float ENCODER_TICK_PER_REV = 600.0;
const float WHEEL_BASE = 100.0;     

float robotX = 0.0;       
float robotY = 0.0;       
float robotHeading = 0.0;  
long initialEnc1 = 0;
long initialEnc2 = 0;
float initialHeadingValue = 0.0;

struct Cell {
  bool visited;
  int order;
  Cell() : visited(false), order(0) {}
};
Cell maze[MAZE_SIZE+5][MAZE_SIZE+5];
int cellOrder = 0;
int currentCellX = -1;
int currentCellY = -1;

const float WALL_THRESHOLD = 30.0;  
const int TURN_ANGLE = 90;

void tcaSelect(uint8_t channel) {
    Wire.beginTransmission(TCA9548A_ADDR);
    Wire.write(1 << channel);
    Wire.endTransmission();
}

bool readButton(int pin) {
  static unsigned long lastDebounceTime[10] = {0};
  static bool buttonState[10] = {HIGH};
  int index = pin;
  bool reading = digitalRead(pin);
  if (reading != buttonState[index]) {
    lastDebounceTime[index] = millis();
  }
  if ((millis() - lastDebounceTime[index]) > 50) {
    buttonState[index] = reading;
  }
  return buttonState[index] == LOW;
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

// Dừng xe
void stopMovement() {
    Right_wheel(DUNG, 0);
    Left_wheel(DUNG, 0);
}

// Xe đi thẳng
void di_thang(int speed) {
    Right_wheel(TIEN, speed);
    Left_wheel(LUI, speed);
}

float angleDifference(float start, float current) {
  float diff = current - start;
  while(diff > 180) diff -= 360;
  while(diff < -180) diff += 360;
  return fabs(diff);
}

void turnRight(int spd, float targetAngle, float startYaw) {
  float currentYaw = getYaw();
  while(angleDifference(startYaw, currentYaw) < targetAngle) {
    Right_wheel(TIEN, spd);
    Left_wheel(TIEN, spd);
    delay(10);
    currentYaw = getYaw();
  }
  stopMovement();
}

// Hàm rẽ trái
void turnLeft(int spd, float targetAngle, float startYaw) {
  float currentYaw = getYaw();
  while(angleDifference(startYaw, currentYaw) < targetAngle) {
    Right_wheel(LUI, spd);
    Left_wheel(LUI, spd);
    delay(10);
    currentYaw = getYaw();
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


void updateRobotPosition() {
  long currentEnc1 = encoder1.getCount();
  long currentEnc2 = encoder2.getCount();
  long deltaEnc1 = currentEnc1 - initialEnc1;
  long deltaEnc2 = currentEnc2 - initialEnc2;
  float dist1 = (deltaEnc1 / ENCODER_TICK_PER_REV) * PI * WHEEL_DIAMETER;
  float dist2 = (deltaEnc2 / ENCODER_TICK_PER_REV) * PI * WHEEL_DIAMETER;
  float deltaHeading = (dist1 - dist2) / WHEEL_BASE;
  robotHeading = initialHeadingValue + deltaHeading * 180 / PI;
  robotHeading = fmod(robotHeading + 360.0, 360.0);
  float distance = (dist1 + dist2) / 2.0;
  robotX += distance * cos(robotHeading * PI / 180.0);
  robotY += distance * sin(robotHeading * PI / 180.0);
}

bool isValidCell(int x, int y) {
  return (x >= 0 && x < MAZE_SIZE && y >= 0 && y < MAZE_SIZE);
}

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
    maze[cellX][cellY].order = ++cellOrder;
    Serial.print("Entered cell: ");
    Serial.print(cellX);
    Serial.print(", ");
    Serial.println(cellY);
  }
  
  // Sensor mapping: dist2 → forward, dist1 → right, dist3 → left.
  bool availForward = (dist2 > WALL_THRESHOLD);
  bool availRight   = (dist1 > WALL_THRESHOLD);
  bool availLeft    = (dist3 > WALL_THRESHOLD);
  
  float h = robotHeading;
  int nx, ny, rx, ry, lx, ly;
  
  if ((h >= 315 || h < 45))         { nx = currentCellX;     ny = currentCellY + 1; }
  else if (h >= 45 && h < 135)        { nx = currentCellX + 1; ny = currentCellY;     }
  else if (h >= 135 && h < 225)       { nx = currentCellX;     ny = currentCellY - 1; }
  else                              { nx = currentCellX - 1; ny = currentCellY;     }
  bool forwardUnvisited = isValidCell(nx, ny) && !maze[nx][ny].visited;
  
  float hRight = fmod(h + 90, 360);
  if ((hRight >= 315 || hRight < 45))  { rx = currentCellX;     ry = currentCellY + 1; }
  else if (hRight >= 45 && hRight < 135){ rx = currentCellX + 1; ry = currentCellY;     }
  else if (hRight >= 135 && hRight < 225){ rx = currentCellX;    ry = currentCellY - 1; }
  else                              { rx = currentCellX - 1; ry = currentCellY;     }
  bool rightUnvisited = isValidCell(rx, ry) && !maze[rx][ry].visited;
  
  float hLeft = fmod(h + 270, 360);
  if ((hLeft >= 315 || hLeft < 45))   { lx = currentCellX;     ly = currentCellY + 1; }
  else if (hLeft >= 45 && hLeft < 135)  { lx = currentCellX + 1; ly = currentCellY;     }
  else if (hLeft >= 135 && hLeft < 225) { lx = currentCellX;     ly = currentCellY - 1; }
  else                              { lx = currentCellX - 1; ly = currentCellY;     }
  bool leftUnvisited = isValidCell(lx, ly) && !maze[lx][ly].visited;
  
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
    if (!cellStack.empty()) {
      std::pair<int,int> prevCell = cellStack.top();
      cellStack.pop();
      Serial.print("Backtracking to cell: ");
      Serial.print(prevCell.first);
      Serial.print(", ");
      Serial.println(prevCell.second);
      int dx = prevCell.first - currentCellX;
      int dy = prevCell.second - currentCellY;
      float targetAngle;
      if (dx == 0 && dy == 1)       targetAngle = 0;
      else if (dx == 1 && dy == 0)  targetAngle = 90;
      else if (dx == 0 && dy == -1) targetAngle = 180;
      else if (dx == -1 && dy == 0) targetAngle = 270;
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

    pinMode(BUTTON1_PIN, INPUT_PULLUP); 
    pinMode(BUTTON2_PIN, INPUT_PULLUP); 

    initialEnc1 = encoder1.getCount();
    initialEnc2 = encoder2.getCount();
    initialHeadingValue = getYaw();

}

void loop() {
  VL53L0X_RangingMeasurementData_t measure1, measure2, measure3;

  tcaSelect(2);
  sensor1.rangingTest(&measure1, false);
  int dist1 = measure1.RangeMilliMeter;

  tcaSelect(6);
  sensor2.rangingTest(&measure2, false);
  int dist2 = measure2.RangeMilliMeter;

  tcaSelect(7);
  sensor3.rangingTest(&measure3, false);
  int dist3 = measure3.RangeMilliMeter;

  updateRobotPosition();

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("Yaw: "); display.println(yaw);
  display.print("Enc1: "); display.println(encoder1.getCount());
  display.print("Enc2: "); display.println(encoder2.getCount());
  display.print("D1: "); display.print(dist1); display.println(" mm");
  display.print("D2: "); display.print(dist2); display.println(" mm");
  display.print("D3: "); display.print(dist3); display.println(" mm");
  display.display();

  Serial.print("Yaw: "); Serial.print(yaw);
  Serial.print(" Enc1: "); Serial.print(encoder1.getCount());
  Serial.print(" Enc2: "); Serial.print(encoder2.getCount());
  Serial.print(" D1: "); Serial.print(dist1);
  Serial.print(" mm D2: "); Serial.print(dist2);
  Serial.print(" mm D3: "); Serial.println(dist3);

  if (readButton(BUTTON1_PIN)) {
    currentMode = (currentMode == IDLE) ? MAPPING : IDLE;
    Serial.println((currentMode == MAPPING) ? "Switching to MAPPING mode" : "Switching to IDLE mode");
    delay(200);
  }
  if (readButton(BUTTON2_PIN)) {
    robotX = 0.0;
    robotY = 0.0;
    initialEnc1 = encoder1.getCount();
    initialEnc2 = encoder2.getCount();
    initialHeadingValue = getYaw();
  }
  
  switch (currentMode) {
    case IDLE:
      stopMovement();
      break;
    case MAPPING:
      dfsDecision(dist1, dist2, dist3);
      break;
  }
  delay(50);
}
