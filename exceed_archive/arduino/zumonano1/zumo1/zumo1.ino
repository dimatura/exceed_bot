//#include <PID_v1.h>
#include <TimedPID.h>

#include <PacketSerial.h>
#include <Wire.h>
#include <Zumo32U4.h>

//#include <CircularBuffer.h>

#define CM_PER_TICK 0.015
#define TICK_PER_CM 66.66
#define WHEEL_BASE_CM 8.5

#define SPEED_MEASURE_WINDOW_MS 40

#define MAX_MOTOR 4000
#define MIN_MOTOR -4000

#define KP 0.05
#define KD 0.0
#define KI 0.001

// right encoder: 2100 ticks/30 cm
// left encoder: 1980 ticks/30 cm
// avg: 2000 ticks/30 cm
// 6666 ticks/m !
// 6666 ticks/s -> 1 m/s which is near max vel
// 3333 ticks -> 0.5 m/s
// 1666 ticks -> 0.25 m/s
// 833 ticks -> 0.125 m/s
// wheel base is 0.085 m

// 200 ticks/s is fairly slow
// 0.2 ticks per ms

struct ZumoSpeed {
  ZumoSpeed() {}
  Zumo32U4Encoders encoders;
  uint32_t last_ms = 0;
  double ticks_per_s[2] = {0., 0.};

  void init() { last_ms = millis(); }

  bool update(uint32_t current_ms) {
    if (current_ms < last_ms) {
      // whatevs
      return false;
    }

    uint32_t elapsed_ms = current_ms - last_ms;

    if (elapsed_ms < SPEED_MEASURE_WINDOW_MS) {
      // time too short, skip
      return false;
    }

    int16_t left_ticks = encoders.getCountsAndResetLeft();
    int16_t right_ticks = encoders.getCountsAndResetRight();

    ticks_per_s[0] = ((double)(left_ticks) * 1e3) / ((double)elapsed_ms);
    ticks_per_s[1] = ((double)(right_ticks) * 1e3) / ((double)elapsed_ms);
    last_ms = current_ms;
    return true;
  }

  double left_ticks_per_s() { return ticks_per_s[0]; }
  double right_ticks_per_s() { return ticks_per_s[1]; }

};

struct ZumoLeds {
  uint8_t yled = 0;
  uint32_t last_ms = 0;

  void init() { last_ms = millis(); }

  void update(uint32_t current_ms) {
    if (current_ms < last_ms) {
      return;
    }

    uint32_t elapsed_ms = current_ms - last_ms;
    if (elapsed_ms < 40) {
      return;
    }

    yled = yled ^ 1;
    ledYellow(yled);
    last_ms = current_ms;
  }
};

struct ZumoPid {

  double target_ticks_per_s[2] = {0., 0.};
  double error[2] = {0., 0.};
  double output[2] = {0., 0.};
  double ticks_per_s[2] = {0., 0.};
  uint32_t last_ms = 0;
  // the PID tells us how much to change speed.
  // so it's the "acceleration".
  // see TimedPID example, in which the PID controls the force applied to a mass to achieve a certain speed.
  // consider, e.g., if error term is 0, then (ignoring I and D), output is 0.
  // alternative (hack?): rely mostly on I term.
  double acceleration = 0.;

  //PID left_pid, right_pid;
  TimedPID left_pid, right_pid;

  ZumoPid() :
    //left_pid(&ticks_per_s[0], &output[0], &target_ticks_per_s[0], KP, KI, KD, DIRECT),
    //right_pid(&ticks_per_s[1], &output[1], &target_ticks_per_s[1], KP, KI, KD, DIRECT)
    left_pid(KP, KI, KD),
    right_pid(KP, KI, KD) {
    //ledGreen(1);
  }

  void init() {
    //left_pid.SetSampleTime(80);
    //right_pid.SetSampleTime(80);
    //left_pid.SetOutputLimits(MIN_MOTOR, MAX_MOTOR);
    //right_pid.SetOutputLimits(MIN_MOTOR, MAX_MOTOR);
    //left_pid.SetMode(AUTOMATIC);
    //right_pid.SetMode(AUTOMATIC);
    //left_pid.setCmdRange(MIN_MOTOR, MAX_MOTOR);
    //right_pid.setCmdRange(MIN_MOTOR, MAX_MOTOR);
    // idk. this is for speed change, not actual motor speed
    left_pid.setCmdRange(-200, 200);
    right_pid.setCmdRange(-200, 200);
  }

  void update(uint32_t current_ms, const ZumoSpeed& zumo_speed, Zumo32U4Motors * motors) {
    if (current_ms < last_ms) {
      return;
    }
    //uint32_t dt_ms = current_ms - last_ms;
    // should we use target or current to calculate?
    // just copy for pid sake. hehe, "pid sake"
    ticks_per_s[0] = zumo_speed.ticks_per_s[0];
    ticks_per_s[1] = zumo_speed.ticks_per_s[1];
    error[0] = target_ticks_per_s[0] - ticks_per_s[0];
    error[1] = target_ticks_per_s[1] - ticks_per_s[1];
    //Serial.print(error[0]);
    //Serial.print(",");
    //Serial.print(error[1]);
    //Serial.print("\n");

    //left_pid.Compute();
    //right_pid.Compute();

    // note we're adding to output
    output[0] += left_pid.getCmdAutoStep(target_ticks_per_s[0], zumo_speed.ticks_per_s[0]);
    output[1] += right_pid.getCmdAutoStep(target_ticks_per_s[1], zumo_speed.ticks_per_s[1]);

    output[0] = constrain(output[0], MIN_MOTOR, MAX_MOTOR);
    output[1] = constrain(output[1], MIN_MOTOR, MAX_MOTOR);

    motors->setSpeeds(output[0], output[1]);
  }
};

Zumo32U4Motors motors;
// Zumo32U4LCD lcd;
// Zumo32U4ButtonA buttonA;
// Zumo32U4ButtonC buttonC;

PacketSerial packets;
ZumoSpeed zumo_speed;
ZumoLeds zumo_leds;
ZumoPid zumo_pid;

void packet_send() {
  double out_buf[8];
  out_buf[0] = zumo_pid.target_ticks_per_s[0];
  out_buf[1] = zumo_pid.target_ticks_per_s[1];  
  out_buf[2] = zumo_speed.ticks_per_s[0];
  out_buf[3] = zumo_speed.ticks_per_s[1];
  out_buf[4] = zumo_pid.error[0];
  out_buf[5] = zumo_pid.error[1];
  out_buf[6] = zumo_pid.output[0];
  out_buf[7] = zumo_pid.output[1];
  packets.send((uint8_t*)(&out_buf[0]), sizeof(out_buf));
}

void on_packet(const uint8_t* buffer, size_t size) {
  if (size < 1) {
    ledYellow(0);
  } else {
    ledYellow(1);
  }
  // note: sizeof(double) in arduino is 4.
  // why do we need the offset??

  memcpy((uint8_t *)zumo_pid.target_ticks_per_s, buffer, size);
  packet_send();
}

void setup() {
  delay(200);
  Serial.begin(115200);
  packets.begin(&Serial);
  packets.setPacketHandler(&on_packet);

  zumo_speed.init();
  zumo_leds.init();
  zumo_pid.init();

  // Serial.setTimeout(1000); // ms

  motors.setSpeeds(0, 0);
  delay(200);
}

void loop() {
  uint32_t current_ms = millis();
  if (zumo_speed.update(current_ms)) {
    zumo_pid.update(current_ms, zumo_speed, &motors);
  }
  zumo_leds.update(current_ms);
  packets.update();
}
