#include <Servo.h>

static constexpr int LED_PIN = 13;
static constexpr int SERVO0_PIN = 3;
static constexpr int SERVO1_PIN = 4;
static constexpr int SERVO2_PIN = 5;

static constexpr int INPUT0_PIN = 0;
static constexpr int INPUT1_PIN = 1;

static constexpr int SERVO_MIN = 40;
static constexpr int SERVO_MAX = 140;

Servo servo0, servo1, servo2;

int servo_pos[3] = {SERVO_MIN, SERVO_MIN, SERVO_MIN};
int servo_delta[3] = {4, 4, 4};

void setup() {
  // initialize the digital pin as an output.
  pinMode(LED_PIN, OUTPUT);
  pinMode(SERVO0_PIN, OUTPUT);
  pinMode(SERVO1_PIN, OUTPUT);
  pinMode(SERVO2_PIN, OUTPUT);
  pinMode(INPUT0_PIN, INPUT);
  pinMode(INPUT1_PIN, INPUT);
  servo0.attach(SERVO0_PIN);
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
}

void loop() {
  int input0 = digitalRead(INPUT0_PIN);
  digitalWrite(LED_PIN, input0);
  for (int i=0; i < 3; ++i) {
    int pos = servo_pos[i];
    if (pos < SERVO_MIN) {
      servo_delta[i] *= -1;
    } else if (pos > SERVO_MAX) {
      servo_delta[i] *= -1;
    }
    servo_pos[i] += servo_delta[i];
  }
  servo0.write(servo_pos[0]);
  servo1.write(servo_pos[1]);
  servo2.write(servo_pos[2]);
  Serial.println("servo: " + String(servo_pos[0]));
  
  delay(80);
  //digitalWrite(LED_PIN, LOW); 
}
