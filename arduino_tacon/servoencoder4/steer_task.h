#ifndef STEER_TASK_H_V9A5BOSI
#define STEER_TASK_H_V9A5BOSI

#include "constants.h"
#include "teensycompat.h"
#include "context.h"


struct SteerControlTask {
  static constexpr int SERVO_CENTER = 90;
  static constexpr int SERVO_MIN = SERVO_CENTER-28;
  static constexpr int SERVO_MAX = SERVO_CENTER+28;
  static constexpr int SERVO_STEER_PIN = 3;
  Servo servo;
  elapsedMillis elapsed_ms;

  void setup() {
    pinMode(SERVO_STEER_PIN, OUTPUT);
    this->servo.attach(SERVO_STEER_PIN);
    this->elapsed_ms = 0;
  }

  void run(GlobalContext* ctx) {
    if (ctx->ms_since_last_input > CMD_TIMEOUT_MS) {
      // stop
      this->servo.write(SERVO_CENTER);
      return;
    }

    if (this->elapsed_ms < STEER_PERIOD_MS) {
        return;
    }

    double inp = constrain(ctx->steer_input_deg, SERVO_MIN, SERVO_MAX);
    this->servo.write(inp);
    this->elapsed_ms = 0;
  }

};


#endif /* end of include guard: STEER_TASK_H_V9A5BOSI */
