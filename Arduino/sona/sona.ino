#define TRIG_PIN 23  // Chân Trigger kết nối với GPIO5 của ESP32
#define ECHO_PIN 2 // Chân Echo kết nối với GPIO18 của ESP32

void setup() {
  Serial.begin(115200);
  
  // Thiết lập các chân Trigger và Echo
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  long duration, distance;

  // Phát xung trên chân Trigger
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Đo thời gian Echo (thời gian sóng âm dội lại)
  duration = pulseIn(ECHO_PIN, HIGH);

  // Tính khoảng cách (tốc độ âm thanh ~ 343 m/s)
  distance = duration * 0.034 / 2;

  // Hiển thị kết quả khoảng cách
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  delay(1000);  // Đợi 1 giây trước khi đo tiếp
}
