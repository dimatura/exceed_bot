
#include <Encoder.h>
#include <CircularBuffer.h>
#include <EasyCommaLib.h>

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

//Encoder enc(ENCA_PIN, ENCB_PIN);
//
//long old_pos = -999;
//elapsedMillis elapsed_ms;

// good for 45
//static constexpr int ENCODER_SAMPLING_PERIOD_MS = 45;
//double ki = 400;
//double kp = 24;

// 60
//static constexpr int ENCODER_SAMPLING_PERIOD_MS = 60;
//double ki = 240;
//double kp = 18;

// 45
static constexpr int ENCODER_SAMPLING_PERIOD_MS = 60;
double ki = 40;
double kp = 8;

//160
//double ki = 24.0;
//double kp = 6.0;

// 80
//static constexpr int ENCODER_SAMPLING_PERIOD_MS = 80;
//double ki = 200;
//double kp = 12; //16;

struct EncoderTask {
  static constexpr int INPUT0_PIN = 0;
  static constexpr int INPUT1_PIN = 1;
  static constexpr double SMOOTH_ALPHA = 0.8;

  // this tpe automagically increases
  elapsedMillis elapsed_ms;
  long elapsed_ms_sum = 0;
  Encoder encoder;

  double ticks_per_s;

  double last_position = 0.0;
  double est_position = 0.0;
  double est_velocity = 0.0;
  double velocity_integrator = 0.0;

  EncoderTask() :
      encoder(INPUT0_PIN, INPUT1_PIN),
      ticks_per_s(0)
  { }

  void setup() {
    elapsed_ms = 0;
    delay(500);
  }

  void run() {
    if (elapsed_ms < ENCODER_SAMPLING_PERIOD_MS) {
      return;
    }

    int32_t new_position = this->encoder.read();
    est_position += ((est_velocity * static_cast<double>(elapsed_ms)) / 1000.0);
    double position_error = (new_position - est_position);
    // 40, 8 works well with velocity_integrator

    velocity_integrator += ((position_error * ki * static_cast<double>(elapsed_ms))/1000.0);
    est_velocity = position_error * kp + velocity_integrator;
    
    //this->ticks_per_s = -(count0_sum_*1000.0)/static_cast<double>(elapsed_ms_sum);
    //this->ticks_per_s = -velocity_integrator;
    this->ticks_per_s = -est_velocity;

    // update global ctx
    //ctx.ticks_per_s = this->ticks_per_s;
    //Serial.println("vel_int:" + String(velocity_integrator) + ", err:" + String(position_error));
    Serial.println("est_pos:" + String(est_position) + ", pos:" + new_position);
    //Serial.println(position_error);
    // reset
    elapsed_ms = 0;
  }

} encoder_task;

void setup() {
  // initialize the digital pin as an output.
  pinMode(LED_PIN, OUTPUT);
}

//EasyComma easyComma(2);
void loop() {
  encoder_task.run();
  /*
  if (Serial.available()) {
    //String ln = Serial.readStringUntil('\n');  
    kp = Serial.parseFloat();
    //ki = Serial.parseFloat();
  }
  //Serial.println(encoder_task.ticks_per_s);
  */
}
