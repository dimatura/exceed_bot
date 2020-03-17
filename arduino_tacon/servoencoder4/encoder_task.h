#ifndef ENCODER_TASK_H_FQWKPAA2
#define ENCODER_TASK_H_FQWKPAA2

#include "constants.h"

#include <CircularBuffer.h>

#include "teensycompat.h"
#include "context.h"

struct EncoderTask {
  static constexpr int INPUT0_PIN = 0;
  static constexpr int INPUT1_PIN = 1;
  static constexpr float SMOOTH_ALPHA = 0.5;
  // TODO: make global and dynamic?
  static constexpr int ENCODER_SAMPLING_PERIOD_MS = 20;
  static constexpr int ENCODER_BUFFER_BUCKETS = 8;

  elapsedMillis elapsed_ms;
  Encoder encoder;
  // prior versions where my encoder was not quadrature used uint32... oops
  CircularBuffer<int32_t, ENCODER_BUFFER_BUCKETS> ticks_buffer;

  //TODO: also look at git history where we try PI-tracking loop idea,
  //from "How to Estimate Encoder Velocity Without Making Stupid Mistakes: Part II"
  //it's clever: that method uses a PI loop where we try to predict
  //next position as current_position + velocity_estimate * dt,
  //and use the error in prediction to adjust velocity estimate.
  //mathematically this was a second order filter.
  //however, on my first few tries it was hard to tune.
	//
	// python code from article:
  //
	//def trkloop(x,dt,kp,ki):
	//		def helper():
	//				velest = 0
	//				posest = 0
	//				velintegrator = 0
	//				for xmeas in x:
	//						posest += velest*dt
	//						poserr = xmeas - posest
	//						velintegrator += poserr * ki * dt
	//						velest = poserr * kp + velintegrator
	//						yield (posest, velest, velintegrator)
	//		y = np.array([yi for yi in helper()])
	//		return y[:,0],y[:,1],y[:,2]
	//[posest,velest,velestfilt] = trkloop(pos_measured,t[1]-t[0],kp=40.0,ki=900.0)

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
    // - I think buffer is overkill for this method... but more flexible. meh
    // fancier:
    // - fit some kind of polynomial (at most a line, realistically) to the buckets, use
    //   the derivative as speed
    //   (see arduinoCurveFitting for polynomials, for line, we can use simple closed form)

    //int32_t ticks = this->encoder.readAndReset();
    int32_t ticks = this->encoder.read();
    // should automatically pop off excess
    this->ticks_buffer.push(ticks);

    int32_t oldest_ticks(this->ticks_buffer.first());
    int32_t newest_ticks(ticks); // also ticks_buffer.tail()

    float delta_ticks = static_cast<float>(newest_ticks - oldest_ticks);
    float delta_ms = static_cast<float>(ENCODER_SAMPLING_PERIOD_MS*this->ticks_buffer.size());

    int32_t ticks_per_s = (delta_ticks*1000.0f)/static_cast<float>(delta_ms);
    // update global ctx (note the minus: forward is positive, and encoder is wired opposite)
    ctx->ticks_per_s = -ticks_per_s;
    elapsed_ms = 0;
  }

  int linreg(size_t n, const float x[], const float y[], float* m, float* b, float* r) {
    float sumx = 0.0f;                      /* sum of x     */
    float sumx2 = 0.0f;                     /* sum of x**2  */
    float sumxy = 0.0f;                     /* sum of x * y */
    float sumy = 0.0f;                      /* sum of y     */
    float sumy2 = 0.0f;                     /* sum of y**2  */

    // TODO: precalculate sumx and sumx2, based on dt
    for (size_t i=0; i < n; i++) {
      sumx  += x[i];
      sumx2 += sq(x[i]);
      sumxy += x[i] * y[i];
      sumy  += y[i];
      sumy2 += sq(y[i]);
    }

    // actually should never happen in this case
    float denom = (n * sumx2 - sq(sumx));
    if (denom < 1e-6) {
      // singular matrix. can't solve the problem.
      *m = 0;
      *b = 0;
      if (r) *r = 0;
      return 1;
    }

    *m = (n * sumxy  -  sumx * sumy) / denom;
    *b = (sumy * sumx2  -  sumx * sumxy) / denom;
    if (r != NULL) {
      *r = (sumxy - sumx * sumy / n) /    /* compute correlation coeff */
          sqrtf((sumx2 - sq(sumx)/n) *
                (sumy2 - sq(sumy)/n));
    }

    return 0;
  }

};


#endif /* end of include guard: ENCODER_TASK_H_FQWKPAA2 */
