
#include <Encoder.h>


static constexpr int LED_PIN = 13;
static constexpr int SERVO0_PIN = 3;
static constexpr int SERVO1_PIN = 4;
static constexpr int SERVO2_PIN = 5;

static constexpr int INPUT0_PIN = 0;
static constexpr int INPUT1_PIN = 1;

static constexpr int SERVO_MIN = 40;
static constexpr int SERVO_MAX = 140;

static constexpr int ENCA_PIN = 0;
static constexpr int ENCB_PIN = 1;

Encoder enc(ENCA_PIN, ENCB_PIN);

long old_pos = -999;
elapsedMillis elapsed_ms;

void setup() {
  // initialize the digital pin as an output.
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  /*
  int input0 = digitalRead(ENCA_PIN);
  int input1 = digitalRead(ENCB_PIN);
  Serial.println(String(input0) + String(input1));
  delay(20);
  */
  /*
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
  //servo2.write(servo_pos[2]);
  */
  if (elapsed_ms > 20) {
    long new_pos = enc.read();
    if (new_pos != old_pos) {
      old_pos = new_pos;
      Serial.println(new_pos);
    }
    elapsed_ms = 0;
  }
  //Serial.println(new_pos);
  //Serial.println("servo: " + String(servo_pos[0]));
  //delay(80);
  //digitalWrite(LED_PIN, LOW); 
}
