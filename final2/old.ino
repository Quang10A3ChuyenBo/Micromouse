#include <Arduino/libraries/ESP32Encoder/src/ESP32Encoder.h>
#include <Arduino/libraries/Adafruit_MPU6050/Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
Adafruit_MPU6050 mpu;
// Biến để tính toán góc Z
float angleZ = 0;
unsigned long lastTime = 0;
#define TRIG_PIN_1 15
#define ECHO_PIN_1 2

#define TRIG_PIN_2 4
#define ECHO_PIN_2 16

#define TRIG_PIN_3 17
#define ECHO_PIN_3 5

// Chân của encoder cho động cơ 1
const int encoderPin1_1 = 32;  // Chân C1 kết nối với GPIO 5
const int encoderPin2_1 = 35; // Chân C2 kết nối với GPIO 18
ESP32Encoder encoder1;

// Chân của encoder cho động cơ 2
const int encoderPin1_2 = 21; // Chân C1 kết nối với GPIO 19
const int encoderPin2_2 = 18; // Chân C2 kết nối với GPIO 21
ESP32Encoder encoder2;
int STBY = 33; //standby

//Motor A
int PWMA = 26; //Speed control
int AIN1 = 13; //Direction
int AIN2 = 14; //Direction+

//Motor B
int PWMB = 25; //Speed control
int BIN1 = 12; //Direction
int BIN2 = 27; //Direction
void setup() {
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  // Khởi tạo MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
     Serial.println("MPU6050 Found!");
    mpu.setGyroRange(MPU6050_RANGE_500_DEG); // Cài đặt dải đo con quay hồi chuyển
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ); // Cài đặt bộ lọc thông thấp
  delay(100);
  
  lastTime = millis(); // Lấy thời gian khởi tạo
   pinMode(TRIG_PIN_1, OUTPUT);
  pinMode(ECHO_PIN_1, INPUT);
  
  pinMode(TRIG_PIN_2, OUTPUT);
  pinMode(ECHO_PIN_2, INPUT);

  pinMode(TRIG_PIN_3, OUTPUT);
  pinMode(ECHO_PIN_3, INPUT);
long getDistance(int trigPin, int echoPin) {
  // Tạo xung trigger dài 10 microseconds
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Đo thời gian phản hồi từ Echo
  long duration = pulseIn(echoPin, HIGH);
  // Tính toán khoảng cách (tốc độ âm thanh là 340m/s)
  long distance = duration * 0.034 / 2;
  return distance;
  pinMode(STBY, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
   Serial.begin(115200);

    // Kết nối encoder cho động cơ 1
    encoder1.attachHalfQuad(encoderPin1_1, encoderPin2_1);

    // Kết nối encoder cho động cơ 2
    encoder2.attachHalfQuad(encoderPin1_2, encoderPin2_2);
    Serial.println("Encoder Test");
}
void loop() {
   // Đọc giá trị từ encoder cho động cơ 1
    long newPosition1 = encoder1.getCount();
    // Đọc giá trị từ encoder cho động cơ 2
    long newPosition2 = encoder2.getCount();
    // In giá trị ra Serial Monitor
    Serial.print("Encoder 1: ");
    Serial.print(newPosition1);
    Serial.print("   Encoder 2: ");
    Serial.println(newPosition2);
    delay(100);
    long distance1 = getDistance(TRIG_PIN_1, ECHO_PIN_1);
  Serial.print("Khoảng cách từ cảm biến 1: ");
  Serial.print(distance1);
  Serial.println(" cm");
  // Đọc khoảng cách từ cảm biến 2
  long distance2 = getDistance(TRIG_PIN_2, ECHO_PIN_2);
  Serial.print("Khoảng cách từ cảm biến 2: ");
  Serial.print(distance2);
  Serial.println(" cm");
  // Đọc khoảng cách từ cảm biến 3
  long distance3 = getDistance(TRIG_PIN_3, ECHO_PIN_3);
  Serial.print("Khoảng cách từ cảm biến 3: ");
  Serial.print(distance3);
  Serial.println(" cm");
  // Tạm dừng 1 giây trước khi lặp lại
  delay(100);
// Cập nhật dữ liệu từ cảm biến MPU6050
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  // Tính thời gian trôi qua từ lần đọc trước
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0; // Tính bằng giây
  lastTime = currentTime;
  // Tính toán góc theo trục Z bằng cách tích lũy vận tốc góc (gyro.z) và chuyển sang độ
  angleZ += g.gyro.z * deltaTime * 180 / PI; // Từ radian/s sang độ
  // // Giới hạn góc Z trong khoảng từ 0 đến 360 độ
   if (angleZ < 0) {
     angleZ += 360;  // Nếu góc âm, cộng thêm 360 để quay lại trong phạm vi 0-360
   } else if (angleZ >= 360) {
   angleZ -= 360;  // Nếu góc lớn hơn hoặc bằng 360, trừ đi 360
   }
  // Hiển thị góc Z theo độ (giới hạn từ 0 đến 360)
  Serial.print("Angle Z: ");
  Serial.println(angleZ);
  
  if (distance1 < 20 && distance2 < 20 && distance3 < 20)
  {
    stop();
    delay(100);

    move(1, 255, 1, 0);
    move(0, 255, 1, 0);
    delay(100);
  }

  if (distance1 < 20 || distance2 < 20 || distance3 < 20) { // Nếu có vật cản gần, dừng lại và quay phải
        stop();
        delay(100);

       turnByAngle(90);

        stop();
        delay(100);
  }
  else
  {
      move(1, 255, 1);
      move(0, 255, 0);
  }

  delay(100);
}
void move(int motor, int speed, int direction, float angle) {
  //Move specific motor at speed and direction
  //motor: 0 for B 1 for A
  //speed: 0 is off, and 255 is full speed
  //direction: 0 clockwise, 1 counter-clockwise

  digitalWrite(STBY, HIGH); //disable standby

  boolean inPin1 = LOW;
  boolean inPin2 = HIGH;

  if (direction == 1) {
    inPin1 = HIGH;
    inPin2 = LOW;
  }

  if (motor == 1) {
    digitalWrite(AIN1, inPin1);
    digitalWrite(AIN2, inPin2);
    analogWrite(PWMA, speed);
  } else {
    digitalWrite(BIN1, inPin1);
    digitalWrite(BIN2, inPin2);
    analogWrite(PWMB, speed);
  }
}

void stop() {
  //enable standby
  digitalWrite(STBY, LOW);
}

void turnByAngle(float targetAngle) {
  float startAngle = angleZ;
  float targetZ = startAngle + targetAngle; 

  if (targetZ >= 360) {
    targetZ -= 360;
  } else if (targetZ < 0) {
    targetZ += 360;
  }

  int direction = (targetAngle > 0) ? 1 : 0;

  // Bắt đầu quay
  move(1, 200, direction); 
  move(0, 200, !direction);

  // Liên tục kiểm tra giá trị angleZ cho đến khi đạt góc mong muốn
  while (true) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Tính thời gian trôi qua từ lần đọc trước
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastTime) / 1000.0; // Tính bằng giây
    lastTime = currentTime;

    // Cập nhật góc Z
    angleZ += g.gyro.z * deltaTime * 180 / PI; // Chuyển từ radian/s sang độ

    // Giới hạn góc Z trong khoảng 0-360 độ
    if (angleZ < 0) {
      angleZ += 360;
    } else if (angleZ >= 360) {
      angleZ -= 360;
    }

    // Hiển thị giá trị góc hiện tại
    Serial.print("Angle Z: ");
    Serial.println(angleZ);

    // Điều kiện để dừng khi đã đạt đến góc mục tiêu
    if (direction == 1) {
      if (angleZ >= targetZ && (angleZ - startAngle) >= targetAngle) {
        break;
      }
    } else { 
      if (angleZ <= targetZ && (startAngle - angleZ) >= -targetAngle) {
        break;
      }
    }
  }

  stop(); // Dừng sau khi quay đủ góc
  delay(500); // Tạm dừng một chút sau khi quay
}
