// Khai báo các chân điều khiển L298N
const int IN1 = 7;  // Chân IN1 của module L298N
const int IN2 = 6;  // Chân IN2 của module L298N
const int ENA = 9;  // Chân ENA để điều chỉnh tốc độ động cơ (PWM)

void setup() {
  // Cấu hình các chân là OUTPUT
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  // Khởi động động cơ
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);  // Động cơ tắt lúc khởi động
}

void loop() {
  // Chạy tiến
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 200);  // Điều chỉnh tốc độ động cơ (0 - 255)
  delay(2000);  // Chạy tiến trong 2 giây

  // Dừng động cơ
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
  delay(1000);  // Dừng 1 giây

  // Chạy lùi
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, 200);
  delay(2000);  // Chạy lùi trong 2 giây

  // Dừng động cơ
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
  delay(1000);  // Dừng 1 giây
}
