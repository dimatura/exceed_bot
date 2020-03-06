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

static constexpr int ENCODER_SAMPLING_PERIOD_MS = 40;

struct EncoderTask {
  elapsedMillis elapsed_ms;
  Encoder encoder;
  double ticks_per_s;
  long old_position;
  
  EncoderTask() : 
  encoder(INPUT0_PIN, INPUT1_PIN),
  ticks_per_s(0),
  old_position(-999)
  {
  }

  void run() {
    if (elapsed_ms < ENCODER_SAMPLING_PERIOD_MS) {
      return;
    }
    long new_position = this->encoder.read();
    // forward is negative
    double delta_ticks = static_cast<double>(-(new_position - old_position));
    double ticks_per_ms = delta_ticks/static_cast<double>(ENCODER_SAMPLING_PERIOD_MS);
    //this->ticks_per_s = 1000.0*ticks_per_ms;
    static constexpr double alpha = 0.8;
    this->ticks_per_s = (1000.0*ticks_per_ms) * alpha + (this->ticks_per_s)*(1.0-alpha);
    old_position = new_position;
    elapsed_ms = 0;
  }

} encoder_task;

struct PrintTask {
  elapsedMillis elapsed_ms;
  
  void run() {
    if (elapsed_ms >= 100) {
      Serial.print(encoder_task.ticks_per_s);
      Serial.print("\n");
      elapsed_ms = 0;
    }
  }
  
} print_task;

PWMServo servo0, servo1;
TimedPID motor_pid(KP, KI, KD);

double out = 0;

// fairly fast speed is 10-20 ticks each 40 ms

void setup() {
  Serial.begin(115200);
  pinMode(SERVO_STEER_PIN, OUTPUT);
  pinMode(SERVO_MOTOR_PIN, OUTPUT);
  // this actually defines servo range so it's weird. seems like esc is 1000-2000ms, 1500 middle.
  // but default of lib is different
  servo1.attach(SERVO_STEER_PIN);
  servo0.attach(SERVO_MOTOR_PIN, 1000, 2000);
  // note that this is delta speed, so accel
  motor_pid.setCmdRange(-10, 10);
}


void loop() {
  encoder_task.run();
  print_task.run();

  /*
  double target = 8;
  double error = (target - delta);
  out += motor_pid.getCmdAutoStep(target, delta);
  out = constrain(out, 90.0, 110.0);
  Serial.println("foo delta: " + String(delta) + ", " + "out: " + String(out) + ", error: " + String(error));

  // motor
  //servo1.write(static_cast<int>(out));
  //servo1.write(90);
  */
}
