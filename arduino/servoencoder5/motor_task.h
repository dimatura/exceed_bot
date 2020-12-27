#ifndef MOTOR_TASK_H_CXFX1U5M
#define MOTOR_TASK_H_CXFX1U5M

#include "constants.h"
#include "teensycompat.h"
#include "context.h"
#include <MiniPID.h>

// we use MiniPID that has feed-forward term. Helps overcome the initial inertia when
// starting, without relying on accurate encoder (which is currently bad at slow speeds),
// and without relying only on error term buildup.

struct MotorControlTask {
  // note for esc we use microseconds, not degrees
  //static constexpr int SERVO_MIN = 90-30;
  //static constexpr int SERVO_MAX = 90+30;
  static constexpr int MOTOR_CENTER_US = 1500;
  static constexpr int SERVO_MIN = MOTOR_CENTER_US-200;
  static constexpr int SERVO_MAX = MOTOR_CENTER_US+200;
  // look into "stop" signal according to robotshop lib - apparently 500 us?

  static constexpr int SERVO_MOTOR_PIN = 4;

  // note that each servo pulse is 20ms
  static constexpr long MOTOR_PERIOD_MS = 40;
  // TODO: maybe only update when encoder updates?
  // note: current tracking pi encoder updates every 20ms

  static constexpr float TICKS_PER_S_DEADZONE = 1.0;

  //PWMServo servo;
  Servo servo;
  //TimedPID motor_pid;
  MiniPID motor_pid;
  elapsedMillis elapsed_ms;

  //MotorControlTask() : motor_pid(DEFAULT_KP, DEFAULT_KI, DEFAULT_KD) { }
  MotorControlTask() : motor_pid(DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, DEFAULT_KF) { }

  void setup() {
    pinMode(SERVO_MOTOR_PIN, OUTPUT);
    // default min, max is 544, 2400
    //this->servo.attach(SERVO_MOTOR_PIN, 1000, 2000);
    this->servo.attach(SERVO_MOTOR_PIN);
    this->servo.writeMicroseconds(MOTOR_CENTER_US);

    // this actually defines servo range so it's weird. seems like esc is 1000-2000ms, 1500 middle.
    // but default of lib is different
    // note that this is delta speed, so accel
    // this->motor_pid.setCmdRange(-10, 10);
    // fairly fast speed is 10-20 ticks each 40 ms -> 250-500 ticks/s
    // 140 ticks/s is also a fairly slow but not too slow pace

    // recall this is norm range, -1, 1
    this->motor_pid.setOutputLimits(-1.0, 1.0);
    // TODO: setMaxIOutput (clamps I term)
    this->elapsed_ms = 0;
  }

  void run(GlobalContext* ctx) {
    if (ctx->pid_gains_reset) {
      this->motor_pid.reset();
      this->motor_pid.setPID(ctx->kp, ctx->ki, ctx->kd, ctx->kf);
      ctx->pid_gains_reset = false;
      return;
    }

    if (ctx->ms_since_last_input > CMD_TIMEOUT_MS) {
      // stop
      ctx->norm_motor_input = 0.0;
      ctx->motor_input_us = MOTOR_CENTER_US;
      this->servo.writeMicroseconds(MOTOR_CENTER_US);
      return;
    }

    if (this->elapsed_ms < MOTOR_PERIOD_MS) {
        return;
    }

    ctx->ticks_error = ctx->target_ticks_per_s - ctx->ticks_per_s;

    if (fabs(ctx->target_ticks_per_s) < TICKS_PER_S_DEADZONE) {
      // do nothing. avoid buildup when stationary
      ctx->norm_motor_input = 0.0;
      ctx->motor_input_us = MOTOR_CENTER_US;
      this->servo.writeMicroseconds(MOTOR_CENTER_US);
      this->elapsed_ms = 0;
      return;
    }

    ctx->norm_motor_input = motor_pid.getOutput(ctx->ticks_per_s, ctx->target_ticks_per_s);

    // unorthodox but fuck it
    // ctx->norm_motor_input = constrain(ctx->norm_motor_input + norm_delta, -1.0, 1.0);

    float out = map(ctx->norm_motor_input, -1.0, 1.0, static_cast<float>(SERVO_MIN), static_cast<float>(SERVO_MAX));
    ctx->motor_input_us = static_cast<int>(out);
    this->servo.writeMicroseconds(static_cast<int>(out));
    //this->servo.writeMicroseconds(MOTOR_CENTER_US);
    this->elapsed_ms = 0;
  }

};



#endif /* end of include guard: MOTOR_TASK_H_CXFX1U5M */
