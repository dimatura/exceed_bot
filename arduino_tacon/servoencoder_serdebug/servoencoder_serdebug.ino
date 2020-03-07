#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <PWMServo.h>
#include <TimedPID.h>
#include <Metro.h>

static constexpr int LED_PIN = 13;
static constexpr int SERVO_STEER_PIN = 3;
static constexpr int SERVO_MOTOR_PIN = 4;
//static constexpr int SERVO2_PIN = 5;

static constexpr int INPUT0_PIN = 0;
static constexpr int INPUT1_PIN = 1;

static constexpr int SERVO_MIN = 40;
static constexpr int SERVO_MAX = 140;

// zumo params
//static constexpr double KP = 0.05;
//static constexpr double KD = 0.0;
//static constexpr double KI = 0.001;

static constexpr double KP = 0.00005;
static constexpr double KD = 0.0;
static constexpr double KI = 0.000001;

/*
Encoder myEnc(INPUT0_PIN, INPUT1_PIN);
PWMServo servo0, servo1;
TimedPID motor_pid(KP, KI, KD);
*/

long oldPosition  = -999;
long delta = 0;
long lastMillis = 0;
double out = 0;

// fairly fast speed is 10-20 ticks each 40 ms

void setup() {
  Serial.begin(9600);
  /*
  pinMode(SERVO_STEER_PIN, OUTPUT);
  pinMode(SERVO_MOTOR_PIN, OUTPUT);
  // this actually defines servo range so it's weird. seems like esc is 1000-2000ms, 1500 middle.
  // but default of lib is different
  servo1.attach(SERVO_STEER_PIN);
  servo0.attach(SERVO_MOTOR_PIN, 1000, 2000);
  lastMillis = millis();
  
  // note that this is delta speed, so accel
  motor_pid.setCmdRange(-10, 10);
  */
}


void loop() {
  double target = 8.0;
  double error = 1.4;
  //out += motor_pid.getCmdAutoStep(target, delta);
  //out = constrain(out, 90.0, 110.0);
  Serial.print("foo delta: ");
  Serial.print(delta); 
  Serial.print(", out: ");
  Serial.print(out);
  Serial.print(", error: ");
  Serial.print(error);
  Serial.print("\n");
  delay(1000);

  // motor
  //servo1.write(static_cast<int>(out));
  //servo1.write(90);
}
