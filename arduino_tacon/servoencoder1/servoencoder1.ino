/* Encoder Library - Basic Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */
   
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <PWMServo.h>
#include <TimedPID.h>

//#include <PulsePosition.h>

static constexpr int LED_PIN = 13;
static constexpr int SERVO0_PIN = 3;
static constexpr int SERVO1_PIN = 4;
static constexpr int SERVO2_PIN = 5;

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


// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder myEnc(INPUT0_PIN, INPUT1_PIN);
PWMServo servo0, servo1;
TimedPID motor_pid(KP, KI, KD);

long oldPosition  = -999;
long delta = 0;
long lastMillis = 0;
double out = 0;

// fairly fast speed is 10-20 ticks each 40 ms

void setup() {
  pinMode(SERVO1_PIN, OUTPUT);
  pinMode(SERVO0_PIN, OUTPUT);
  //pinMode(5, OUTPUT);
  // this actually defines servo range so it's weird. seems like esc is 1000-2000ms, 1500 middle.
  // but default of lib is different
  servo1.attach(SERVO1_PIN, 1000, 2000);
  servo0.attach(SERVO0_PIN);
  lastMillis = millis();
  Serial.begin(115200);

  // notee that this is delta speed, so accel
  motor_pid.setCmdRange(-10, 10);
  
}


void loop() {
  
  long currentMillis = millis();
  if ((currentMillis - lastMillis) >= 40) {
    long newPosition = myEnc.read();
    // forward is negative
    delta = -(newPosition - oldPosition);
    oldPosition = newPosition;
    lastMillis = currentMillis;
    //Serial.println(delta);
  }

  double target = 8;
  double error = (target - delta);
  out += motor_pid.getCmdAutoStep(target, delta);
  out = constrain(out, 90.0, 110.0);
  Serial.println("delta: " + String(delta) + ", " + "out: " + String(out) + ", error: " + String(error));
 
  // motor
  servo1.write(static_cast<int>(out));
  //servo1.write(90);
}
