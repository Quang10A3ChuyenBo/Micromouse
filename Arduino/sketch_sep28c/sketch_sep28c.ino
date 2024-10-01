#include <ESP32Encoder.h>

// Chân của encoder cho động cơ 1
const int encoderPin1_1 = 32;  // Chân C1 kết nối với GPIO 5
const int encoderPin2_1 = 35; // Chân C2 kết nối với GPIO 18
ESP32Encoder encoder1;

// Chân của encoder cho động cơ 2
const int encoderPin1_2 = 34; // Chân C1 kết nối với GPIO 19
const int encoderPin2_2 = 18;// Chân C2 kết nối với GPIO 21
ESP32Encoder encoder2;
#define TRIG_PIN_1 4
#define ECHO_PIN_1 5

#define TRIG_PIN_2 15
#define ECHO_PIN_2 19

#define TRIG_PIN_3 23
#define ECHO_PIN_3 2
long readUltrasonicDistance(int trigPin, int echoPin) {
digitalWrite(trigPin, LOW);
delayMicroseconds(2);
digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);

long duration = pulseIn(echoPin, HIGH);
long distance = duration * 0.034 / 2;
return distance;
}
void setup() {
Serial.begin(115200);

// Kết nối encoder cho động cơ 1
encoder1.attachHalfQuad(encoderPin1_1, encoderPin2_1);

// Kết nối encoder cho động cơ 2
encoder2.attachHalfQuad(encoderPin1_2, encoderPin2_2);

Serial.println("Encoder Test");
// put your setup code here, to run once:
Serial.begin(115200);

pinMode(TRIG_PIN_1, OUTPUT);
pinMode(ECHO_PIN_1, INPUT);

pinMode(TRIG_PIN_2, OUTPUT);
pinMode(ECHO_PIN_2, INPUT);

pinMode(TRIG_PIN_3, OUTPUT);
pinMode(ECHO_PIN_3, INPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.begin(115200);

  pinMode(TRIG_PIN_1, OUTPUT);
  pinMode(ECHO_PIN_1, INPUT);

  pinMode(TRIG_PIN_2, OUTPUT);
  pinMode(ECHO_PIN_2, INPUT);

  pinMode(TRIG_PIN_3, OUTPUT);
  pinMode(ECHO_PIN_3, INPUT);

}
