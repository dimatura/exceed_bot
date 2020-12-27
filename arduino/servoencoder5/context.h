#ifndef CONTEXT_H_RD4JOWIK
#define CONTEXT_H_RD4JOWIK

#include "constants.h"

#if ON_TEENSY
#else
#include <elapsedMillis.h>
#endif

struct GlobalContext {
  float ticks_per_s = 0.0;
  int raw_ticks = 0;
  float target_ticks_per_s = 0.0;
  float ticks_error = 0.0;
  int steer_input_deg = 90;
  float norm_motor_input = 0.0;
  int motor_input_us = 90;

  float kp = DEFAULT_KP;
  float ki = DEFAULT_KI;
  float kd = DEFAULT_KD;
  float kf = DEFAULT_KF;
  bool pid_gains_reset = false;

  elapsedMillis ms_since_last_input;
};

#endif /* end of include guard: CONTEXT_H_RD4JOWIK */
