#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>         mm   
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

const int BUTTON1_PIN = ;

// Enums for Modes
enum RobotMode {
  IDLE,  
  MAPPING, 
  PATHFIND
};

RobotMode currentMode = IDLE;


const float WALL_THRESHOLD = 30.0;
const int TURN_ANGLE = 90;
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
    bool northWall;
    bool southWall;
    bool eastWall;
    bool westWall;
    bool visited;
    int floodFillValue;
    float x; 
    float y;

    Cell() : northWall(false), southWall(false), eastWall(false), westWall(false),
             visited(false), floodFillValue(-1), x(0.0), y(0.0) {}
};
Cell maze[MAZE_SIZE+5][MAZE_SIZE+5];



void tcaSelect(uint8_t channel) {
    Wire.beginTransmission(TCA9548A_ADDR);
    Wire.write(1 << channel);
    Wire.endTransmission();
}

bool readButton(int pin) {
  static unsigned long lastDebounceTime[2] = {0, 0};  // Debounce time for each button
  static bool buttonState[2] = {HIGH, HIGH};          // Current button state (HIGH = not pressed)
  int buttonIndex = (pin == BUTTON1_PIN) ? 0 : 1;   // Determine the index for the button

  bool reading = digitalRead(pin); // Read the raw button input

  // If the switch changed, due to noise or pressing:
  if (reading != buttonState[buttonIndex]) {
    // Reset the debouncing timer
    lastDebounceTime[buttonIndex] = millis();
  }

  if ((millis() - lastDebounceTime[buttonIndex]) > 50) { // Debounce delay of 50ms
    // Whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:
    buttonState[buttonIndex] = reading;
  }

  return buttonState[buttonIndex] == LOW; // Return true if button is pressed (LOW)
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

long prevEnc1 = 0;
long prevEnc2 = 0;

void updateRobotPosition() {
    long currentEnc1 = encoder1.getCount();
    long currentEnc2 = encoder2.getCount();

    long deltaEnc1 = currentEnc1 - initialEnc1;
    long deltaEnc2 = currentEnc2 - initialEnc2;

    float dist1 = (deltaEnc1 / ENCODER_TICK_PER_REV) * PI * WHEEL_DIAMETER;
    float dist2 = (deltaEnc2 / ENCODER_TICK_PER_REV) * PI * WHEEL_DIAMETER;

    float deltaHeading = (dist1 - dist2) / WHEEL_BASE; // in radians

    robotHeading = initialHeadingValue + deltaHeading * 180 / PI;

    robotHeading = fmod(robotHeading + 360.0, 360.0);

    float distance = (dist1 + dist2) / 2.0;

    robotX += distance * cos(robotHeading * PI / 180.0);
    robotY += distance * sin(robotHeading * PI / 180.0);
}

void updateMap(int dist1, int dist2, int dist3) {
    int currentCellX = (int)((robotX + 1200) / CELL_SIZE);
    int currentCellY = (int)((robotY + 1200) / CELL_SIZE);

    if (currentCellX < 0 || currentCellX >= MAZE_SIZE || currentCellY < 0 || currentCellY >= MAZE_SIZE) {
        return;
    }
    maze[currentCellX][currentCellY].visited = true;

    //Update walls
    maze[currentCellX][currentCellY].eastWall = (dist1 <= WALL_THRESHOLD);
    maze[currentCellX][currentCellY].northWall = (dist2 <= WALL_THRESHOLD);
    maze[currentCellX][currentCellY].westWall = (dist3 <= WALL_THRESHOLD);
}

// ==================================================================
// A* Search Algorithm
// ==================================================================
struct Node {
    int x;
    int y;
    int f;
    int g;
    int h;
    Node *parent;
};

int calculateHeuristic(int x1, int y1, int x2, int y2) {
    return abs(x1 - x2) + abs(y1 - y2);
}

bool isValidCell(int x, int y) {
    return (x >= 0 && x < MAZE_SIZE && y >= 0 && y < MAZE_SIZE);
}

bool isTraversable(int x, int y, int direction) {
    if (!isValidCell(x, y)) return false;

    switch (direction) {
        case 0: // North
            return !maze[x][y].northWall;
        case 1: // East
            return !maze[x][y].eastWall;
        case 2: // South
            return !maze[x][y].southWall;
        case 3: // West
            return !maze[x][y].westWall;
        default:
            return false;
    }
}

std::vector<std::pair<int, int>> aStarSearch(int startX, int startY, int goalX, int goalY) {
    std::vector<std::pair<int, int> > path;
    return path;
}

void followPath(std::vector<std::pair<int, int>> path) {
    Serial.println("Following A* path:");
    for (const auto& cell : path) {
        int targetCellX = cell.first;
        int targetCellY = cell.second;

        Serial.print("Moving to cell: (");
        Serial.print(targetCellX);
        Serial.print(", ");
        Serial.print(targetCellY);
        Serial.println(")");

        float targetAngle = atan2((targetCellY * CELL_SIZE) - robotY, (targetCellX * CELL_SIZE) - robotX) * 180 / PI;
        float angleDiff = targetAngle - robotHeading;

        while (angleDiff > 180) angleDiff -= 360;
        while (angleDiff <= -180) angleDiff += 360;

        Serial.print("Turning by: "); Serial.println(angleDiff);

        if (angleDiff > 5) {
            turnRight(speed, angleDiff, getYaw());
        } else if (angleDiff < -5) {
            turnLeft(speed, -angleDiff, getYaw());
        }

        float distanceToCell = sqrt(pow((targetCellX * CELL_SIZE) - robotX, 2) + pow((targetCellY * CELL_SIZE) - robotY, 2));
        Serial.print("Moving forward by: "); Serial.println(distanceToCell);
        di_thang(speed); 
    }
        Serial.println("Arrived!");

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

    for (int i = 0; i < MAZE_SIZE; i++) {
        for (int j = 0; j < MAZE_SIZE; j++) {
            maze[i][j].x = i * CELL_SIZE;
            maze[i][j].y = j * CELL_SIZE;
        }
    }
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

    if (readButton(BUTTON1_PIN)) {
       switch (currentMode) {
          case IDLE:
            currentMode = MAPPING;
            Serial.println("Switching to MAPPING mode");
            break;
          case MAPPING:
            currentMode = PATHFIND;
            Serial.println("Switching to PATHFIND mode");
            break;
          case PATHFIND:
            currentMode = IDLE;
            Serial.println("Switching to IDLE mode");
            break;  
        }
        delay(200);
      }

      if (readButton(BUTTON2_PIN)) {
            robotX = 0.0;
            robotY = 0.0;
            initialEnc1 = encoder1.getCount();
            initialEnc2 = encoder2.getCount();
            initialHeadingValue = getYaw();
        }
    
    long enc1 = encoder1.getCount();
    long enc2 = encoder2.getCount();

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print("Yaw(Z): "); display.println(yaw);
    display.print(" Enc1: "); display.print(enc1);
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

    updateRobotPosition();
        switch (currentMode) {
          case IDLE:
            stopMovement();
            break;
          case MAPPING:
            updateMap(dist1, dist2, dist3);
            di_thang(speed);

            break;
          case PATHFIND:
              std::vector<std::pair<int, int>> path = aStarSearch(0,0,MAZE_SIZE-1, MAZE_SIZE-1);
              followPath(path);
            break;
          case MANUAL:
            updateRobotPosition();
            if(digitalRead(19) == HIGH) {di_thang(speed); Serial.println("Doing what ENC1 want");}
            else stopMovement();

            break;
        }

    delay(50);
}
