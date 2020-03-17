#include <TimedPID.h>
#include <CircularBuffer.h>

#include "constants.h"

#include "comms_task.h"
#include "encoder_task.h"
#include "motor_task.h"
#include "steer_task.h"
#include "context.h"


struct BlinkTask {
  static constexpr int LED_PIN = 13;
  elapsedMillis elapsed_ms;
  int led_state = 0;

  void setup() {
    //Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);
  }

  void run(GlobalContext* ctx) {
    if (elapsed_ms >= 250) {
      digitalWrite(LED_PIN, this->led_state);
      this->led_state = 1 - this->led_state;
      this->elapsed_ms = 0;
    }
  }
};


GlobalContext ctx;
BlinkTask blink_task;
CommsTask comms_task;
EncoderTask encoder_task;
MotorControlTask motor_control_task;
SteerControlTask steer_control_task;


void setup() {
  encoder_task.setup();
  motor_control_task.setup();
  steer_control_task.setup();
  comms_task.setup();
  blink_task.setup();
}


void loop() {
  blink_task.run(&ctx);
  encoder_task.run(&ctx);
  motor_control_task.run(&ctx);
  steer_control_task.run(&ctx);
  comms_task.run(&ctx);
}

//vim: filetype=cpp:
