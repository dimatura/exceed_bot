
#include <Encoder.h>
#include <CircularBuffer.h>


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

struct EncoderTask {
  static constexpr int INPUT0_PIN = 0;
  static constexpr int INPUT1_PIN = 1;
  static constexpr double SMOOTH_ALPHA = 0.8;
  static constexpr int ENCODER_SAMPLING_PERIOD_MS = 80;
  static constexpr int ENCODER_BUFFER_SIZE = 6;
  static constexpr double BUF_COUNT_TO_TICKS_PER_S = 1000.0/static_cast<double>(ENCODER_SAMPLING_PERIOD_MS*ENCODER_BUFFER_SIZE);
  // this tpe automagically increases
  elapsedMillis elapsed_ms;
  long elapsed_ms_sum = 0;
  Encoder encoder;

  double ticks_per_s;

  int32_t count0_sum_ = 0;
  CircularBuffer<int32_t, ENCODER_BUFFER_SIZE> counts_;
  CircularBuffer<int32_t, ENCODER_BUFFER_SIZE> elapsed_ms_buf_;

  EncoderTask() :
      encoder(INPUT0_PIN, INPUT1_PIN),
      ticks_per_s(0)
  { }

  void setup() {
    elapsed_ms = 0;
  }

  void run() {
    if (elapsed_ms < ENCODER_SAMPLING_PERIOD_MS) {
      return;
    }

    int32_t ct0 = this->encoder.readAndReset();
    count0_sum_ += ct0;
    elapsed_ms_sum += elapsed_ms;
    
    if (counts_.isFull()) {
      // pop out last item, add it to moving sum
      count0_sum_ -= counts_.pop();
      elapsed_ms_sum -= elapsed_ms_buf_.pop();
    }
    // should always have at least one slot
    counts_.push(ct0);
    elapsed_ms_buf_.push(elapsed_ms);

    //long new_position = this->encoder.read();
    //double delta_ticks = new_position - old_position;

    // forward is negative
    //this->ticks_per_s = -count0_sum_*BUF_COUNT_TO_TICKS_PER_S;
    this->ticks_per_s = -(count0_sum_*1000.0)/static_cast<double>(elapsed_ms_sum);
    //this->ticks_per_s = count0_sum_;

    // update global ctx
    //ctx.ticks_per_s = this->ticks_per_s;
    Serial.println(this->ticks_per_s);
    // reset
    elapsed_ms = 0;
  }

} encoder_task;

void setup() {
  // initialize the digital pin as an output.
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  encoder_task.run();
  //Serial.println(encoder_task.ticks_per_s);
}
