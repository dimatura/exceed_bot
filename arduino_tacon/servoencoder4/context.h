#ifndef CONTEXT_H_RD4JOWIK
#define CONTEXT_H_RD4JOWIK

#include "constants.h"

#if ON_TEENSY
#else
#include <elapsedMillis.h>
#endif

struct GlobalContext {
  double ticks_per_s = 0.0;
  double target_ticks_per_s = 0.0;
  double ticks_error = 0.0;
  int steer_input_deg = 90;
  double norm_motor_input = 0.0;
  int motor_input_us = 90;

  double kp = DEFAULT_KP;
  double ki = DEFAULT_KI;
  double kd = DEFAULT_KD;
  bool pid_gains_reset = false;

  elapsedMillis ms_since_last_input;
};

#endif /* end of include guard: CONTEXT_H_RD4JOWIK */
