//#include <Metro.h>
#include <CircularBuffer.h>
#include <PacketSerial.h>

#define PIN_ENC_0 0
#define PIN_LED 13

#define LED_INTERVAL_MS 250
// if we use 50 and 20 then window will be 1s
#define SPEED_INTERVAL_MS 50
#define SPEED_BUFFER_SIZE 20

// note: exceedrc makes 90 ticks per 30 cm. (roughly, but number nice and round).
#define TICKS_PER_M 300

// encoder stuff
// global
//volatile uint32_t count0 = 0;
volatile byte count0 = 0;

class LedTask {
 private:
  // led status
  byte led_ = 0;

  uint32_t last_ms_ = 0;

 public:
  void init() {
    last_ms_ = millis();
  }

  bool update(uint32_t ms) {
    uint32_t elapsed = ms - last_ms_;
    if (elapsed > LED_INTERVAL_MS) {
      led_ = 1 - led_;
      digitalWrite(PIN_LED, led_);
      last_ms_ = ms;
      return true;
    }
    return false;
  }
};


class SpeedTask {
 private:
  // using appropriate intervals this is ticks per s
  uint32_t count0_sum_ = 0;
  CircularBuffer<uint32_t, SPEED_BUFFER_SIZE> counts_;
  uint32_t last_ms_ = 0;

 public:
  void init() {
    last_ms_ = millis();
  }

  bool update(uint32_t ms) {
    uint32_t elapsed = ms - last_ms_;
    if (elapsed > SPEED_INTERVAL_MS) {
      noInterrupts();
      uint32_t ct0 = count0;
      count0 = 0;
      interrupts();

      count0_sum_ += ct0;
      if (counts_.isFull()) {
        // pop out last item, add it to moving sum
        count0_sum_ -= counts_.pop();
      }
      // should always have at least one slot
      counts_.push(ct0);

      last_ms_ = ms;
      return true;
    }
    return false;
  }

  uint32_t count0_sum() {
    return count0_sum_;
  }
};


PacketSerial packets;
LedTask led_task;
SpeedTask speed_task;

byte led;

void on_enc0_change() {
  count0++;
  //led = 1 - led;
  //digitalWrite(PIN_LED, led);
}

void on_packet(const uint8_t * buffer, size_t size) {
  // TODO
}

void send_status() {
  uint8_t buff[sizeof(uint32_t)];
  uint32_t count_sum = speed_task.count0_sum();
  memcpy(&buff[0], (uint8_t *) &count_sum, sizeof(buff));
  packets.send(buff, sizeof(buff));
}

void setup() {
  Serial.begin(115200);
  packets.setStream(&Serial);
  packets.setPacketHandler(&on_packet);

  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_ENC_0, INPUT);
  // LOW, CHANGE, RISING, FALLING, HIGH
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_0), on_enc0_change, CHANGE);

  led_task.init();
  speed_task.init();
}


void loop() {

  uint32_t ms = millis();

  led_task.update(ms);

  if (speed_task.update(ms)) {
    send_status();
  }

  packets.update();
}
