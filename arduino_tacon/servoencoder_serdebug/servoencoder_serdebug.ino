#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <CircularBuffer.h>

#if 0
#include <TimedPID.h>
#include <PWMServo.h>
#endif

static constexpr int LED_PIN = 13;
static constexpr int SERVO_STEER_PIN = 3;
static constexpr int SERVO_MOTOR_PIN = 4;
//static constexpr int SERVO2_PIN = 5;

static constexpr int INPUT0_PIN = 0;
static constexpr int INPUT1_PIN = 1;

Encoder myEnc(INPUT0_PIN, INPUT1_PIN);

long oldPosition  = -999;
long delta = 0;
long lastMillis = 0;
double out = 0;

elapsedMillis start_ms;
elapsedMillis encoder_sampling_ms;
int led_state = 0;

struct TicksEntry {
  unsigned long ms;
  int32_t ticks;
};

CircularBuffer<TicksEntry, 10> buffer;

void setup() {
  Serial.begin(115200);
  start_ms = 0;
  encoder_sampling_ms = 0;
  pinMode(LED_PIN, OUTPUT);
}


void loop() {
  if (encoder_sampling_ms < 20) {
    return;
  }
  encoder_sampling_ms = 0;

  int32_t ticks = myEnc.read();
  //int32_t ticks = 0;
  digitalWrite(LED_PIN, led_state);
  led_state = 1 - led_state;

  TicksEntry entry;
  entry.ms = static_cast<unsigned long>(start_ms);
  entry.ticks = ticks;
  buffer.push(entry);

  if (buffer.isFull()) {
    while (!buffer.isEmpty()) {
      TicksEntry entry = buffer.pop();
      Serial.print(static_cast<unsigned long>(entry.ms));
      Serial.print(",");
      Serial.print(entry.ticks);
      //Serial.print(0);
      Serial.print("\n");
    }
  }
}
