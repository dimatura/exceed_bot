#ifndef ENCODER_TASK_H_FQWKPAA2
#define ENCODER_TASK_H_FQWKPAA2

#include "constants.h"
#include "teensycompat.h"
#include "context.h"

struct EncoderTask {
  static constexpr int INPUT0_PIN = 0;
  static constexpr int INPUT1_PIN = 1;
  static constexpr double SMOOTH_ALPHA = 0.5;

  elapsedMillis elapsed_ms;
  Encoder encoder;

  double last_ticks_per_s;
  //double last_position = 0.0;

  EncoderTask() :
      encoder(INPUT0_PIN, INPUT1_PIN),
      last_ticks_per_s(0)
  { }

  void setup() {
    this->elapsed_ms = 0;
  }

  void run(GlobalContext* ctx) {
    if (this->elapsed_ms < ENCODER_SAMPLING_PERIOD_MS) {
      return;
    }

    int32_t ticks = this->encoder.readAndReset();
    int32_t ticks_per_s = -(ticks*1000.0)/static_cast<double>(this->elapsed_ms);

    // update global ctx
    ctx->ticks_per_s = (ticks_per_s + this->last_ticks_per_s)/2;
    //ctx->ticks_per_s = ticks_per_s;

    this->last_ticks_per_s = ticks_per_s;
    elapsed_ms = 0;
  }

};


#endif /* end of include guard: ENCODER_TASK_H_FQWKPAA2 */
