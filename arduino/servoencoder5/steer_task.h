#ifndef STEER_TASK_H_V9A5BOSI
#define STEER_TASK_H_V9A5BOSI

#include "constants.h"
#include "teensycompat.h"
#include "context.h"


struct SteerControlTask {
  // note; 95 is based on tacon, using 90 makes it go to the left
  static constexpr int SERVO_CENTER_DEG = 95;
  static constexpr int SERVO_MIN = SERVO_CENTER_DEG-28;
  static constexpr int SERVO_MAX = SERVO_CENTER_DEG+28;
  static constexpr int SERVO_STEER_PIN = 3;
  static constexpr int SERVO_DEADZONE_DEG = 2;

  // note that each servo pulse is 20ms
  static constexpr long STEER_PERIOD_MS = 40;

  Servo servo;
  elapsedMillis elapsed_ms;

  void setup() {
    pinMode(SERVO_STEER_PIN, OUTPUT);
    this->servo.attach(SERVO_STEER_PIN);
    this->servo.write(SERVO_CENTER_DEG);
    this->elapsed_ms = 0;
  }

  void run(GlobalContext* ctx) {
    if (ctx->ms_since_last_input > CMD_TIMEOUT_MS) {
      // turns out tacon servo just holds position when it gets no pulses
      // so detach does nothing
      // if (this->servo.attached()) { this->servo.detach(); }
      // this method doesn't do anything either
      // this->servo.writeMicroseconds(500);
      this->servo.write(SERVO_CENTER_DEG);
      return;
    }

    if (this->elapsed_ms < STEER_PERIOD_MS) {
      return;
    }

    if (!this->servo.attached()) {
      this->servo.attach(SERVO_STEER_PIN);
    }

    float inp = constrain(ctx->steer_input_deg, SERVO_MIN, SERVO_MAX);
    // TODO detach instead?
    // if ((inp >= -SERVO_DEADZONE_DEG) && (inp <= SERVO_DEADZONE_DEG)) { return; }
    this->servo.write(inp);
    this->elapsed_ms = 0;
  }

};


#endif /* end of include guard: STEER_TASK_H_V9A5BOSI */
