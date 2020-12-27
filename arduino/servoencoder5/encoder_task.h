#ifndef ENCODER_TASK_H_FQWKPAA2
#define ENCODER_TASK_H_FQWKPAA2

#include "constants.h"

#include <CircularBuffer.h>

#include "teensycompat.h"
#include "context.h"


struct EncoderTask {
  static constexpr int INPUT0_PIN = 0;
  static constexpr int INPUT1_PIN = 1;
  //static constexpr float SMOOTH_ALPHA = 0.5;
  // TODO: make global and dynamic?
  static constexpr int ENCODER_SAMPLING_PERIOD_MS = 20;
  static constexpr int ENCODER_BUFFER_BUCKETS = 8;

  //static constexpr float TRACK_KP = 4.0;
  //static constexpr float TRACK_KI = 80.0;
  static constexpr float TRACK_KP = 8.0;
  static constexpr float TRACK_KI = 80.0;

  elapsedMillis elapsed_ms;
  Encoder encoder;
  // prior versions where my encoder was not quadrature used uint32... oops
  // CircularBuffer<int32_t, ENCODER_BUFFER_BUCKETS> ticks_buffer;

  // see: servoencoder_serdebug/encoder_analysis1.py
  // tracking loop from
  // "How to Estimate Encoder Velocity Without Making Stupid Mistakes: Part II"
  // idea: a PI loop where we try to predict
  // next position as current_position + velocity_estimate * dt,
  // and use the error in prediction to adjust velocity estimate.
  // mathematically this is a second order filter.
  // note: I suspect adding feed-forward to controller will compensate for lower accuracy at slow speed

  float vel_est = 0.0; // PI velocity estimate
  float ticks_est = 0.0; // predicted ticks
  float vel_int = 0.0; // integrated velocity, smoother but slower than vel_est

  EncoderTask() :
      encoder(INPUT0_PIN, INPUT1_PIN)
  { }

  void setup() {
    this->elapsed_ms = 0;
  }

  void run(GlobalContext* ctx) {
    if (this->elapsed_ms < ENCODER_SAMPLING_PERIOD_MS) {
      return;
    }
    // two (I think) equivalent ways:
    // - readAndReset each time, keep running count as we push and pop
    // - just read, then subtract last from first in buffer

    //int32_t ticks = this->encoder.readAndReset();
    int32_t ticks = this->encoder.read();

    /*
    this->ticks_buffer.push(ticks); // automatically pops off oldest
    int32_t oldest_ticks(this->ticks_buffer.first());
    int32_t newest_ticks(ticks); // also ticks_buffer.tail()

    float delta_ticks = static_cast<float>(newest_ticks - oldest_ticks);
    float delta_ms = static_cast<float>(ENCODER_SAMPLING_PERIOD_MS*this->ticks_buffer.size());

    int32_t ticks_per_s = (delta_ticks*1000.0f)/static_cast<float>(delta_ms);
    // update global ctx (note the minus: forward is positive, and encoder is wired opposite)
    ctx->ticks_per_s = -ticks_per_s;
    */

    // TODO: clamping for I term? adaptive I term for slower speed?
    float dt_s = static_cast<float>(this->elapsed_ms)/1000.0f; // could be just assumed to be 20ms
    this->ticks_est += this->vel_est * dt_s; // predict ticks we should see re: vel_est
    float ticks_err = static_cast<float>(ticks) - this->ticks_est; // compute error in prediction
    this->vel_int += (ticks_err * TRACK_KI * dt_s); // wind up the I-term
    this->vel_est = ticks_err * TRACK_KP + this->vel_int; // new vel_est: P-term plus I-term
    ctx->ticks_per_s = -this->vel_est;

    // raw ticks for post-hoc analysis
    ctx->raw_ticks = ticks;

    elapsed_ms = 0;
  }

};


#endif /* end of include guard: ENCODER_TASK_H_FQWKPAA2 */
