#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

// Biến để tính toán góc Z
float angleZ = 0;
unsigned long lastTime = 0;

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  // Khởi tạo MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setGyroRange(MPU6050_RANGE_500_DEG); // Cài đặt dải đo con quay hồi chuyển
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ); // Cài đặt bộ lọc thông thấp
  delay(100);
  
  lastTime = millis(); // Lấy thời gian khởi tạo
}

void loop() {
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

  delay(1000); // Điều chỉnh thời gian lấy mẫu
}
