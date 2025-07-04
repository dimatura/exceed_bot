#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <TimedPID.h>
#include <ArduinoJson.h>
#include <CircularBuffer.h>
//#include <PacketSerial.h>
#include <Servo.h>
//#include <PWMServo.h>
//#include "/home/dmaturan/apps/arduino-1.8.12/hardware/teensy/avr/libraries/Servo/Servo.h"


static constexpr int LED_PIN = 13;
//static constexpr int SERVO2_PIN = 5;

//static constexpr double DEFAULT_KP = 0.00005;
//static constexpr double DEFAULT_KD = 0.0;
//static constexpr double DEFAULT_KI = 0.000001;

static constexpr double DEFAULT_KP = 6e-4;
static constexpr double DEFAULT_KD = 0.00;
static constexpr double DEFAULT_KI = 0.00;

static constexpr long CMD_TIMEOUT_MS = 2000;
static constexpr int ENCODER_SAMPLING_PERIOD_MS = 60;
static constexpr int STEER_PERIOD_MS = ENCODER_SAMPLING_PERIOD_MS;
static constexpr int MOTOR_PERIOD_MS = ENCODER_SAMPLING_PERIOD_MS;


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
} ctx;


struct CommsTask {
  static constexpr size_t BUF_LEN = 1024;
  uint8_t input_buf[BUF_LEN];
  uint8_t output_buf[BUF_LEN];
  size_t inp_buf_ix = 0;

  static const int input_msg_capacity = JSON_OBJECT_SIZE(2) + 80;
  static const int output_msg_capacity = JSON_OBJECT_SIZE(6) + 120;

  bool error = false;
  bool overflow = false;

  CommsTask() {
    for (size_t i = 0; i < BUF_LEN; ++i) {
      input_buf[i] = '\0';
      output_buf[i] = '\0';
    }
  }

  void setup() {
    Serial.begin(115200);
    //this->packets.begin(&Serial);
    //this->packets.setPacketHandler(&on_packet);
  }

  bool read_msg() {
    //int buf_read = Serial.readBytesUntil('\n', input_buf, BUF_LEN - 1);
    StaticJsonDocument<input_msg_capacity> input_doc;
    bool received = false;
    while (Serial.available() > 0) {
      uint8_t data = Serial.read();
      if (data == '\n') {
        // decode
        DeserializationError err = deserializeJson(input_doc, this->input_buf, BUF_LEN-1);
        this->error = false;
        if (err == DeserializationError::Ok) {
          JsonVariant tticks_var = input_doc["target_ticks_per_s"];
          if (!tticks_var.isNull()) {
            ctx.target_ticks_per_s = tticks_var.as<double>();
          } else {
            this->error = true;
          }

          JsonVariant steer_input_deg_var = input_doc["target_steer_deg"];
          if (!steer_input_deg_var.isNull()) {
            ctx.steer_input_deg = steer_input_deg_var.as<int>();
          } else {
            this->error = true;
          }

          // these are optional
          JsonVariant kp = input_doc["kp"];
          if (!kp.isNull()) {
            ctx.kp = kp.as<double>();
            ctx.pid_gains_reset = true;
          }
          JsonVariant ki = input_doc["ki"];
          if (!ki.isNull()) {
            ctx.ki = ki.as<double>();
            ctx.pid_gains_reset = true;
          }
          JsonVariant kd = input_doc["kd"];
          if (!kd.isNull()) {
            ctx.kd = kd.as<double>();
            ctx.pid_gains_reset = true;
          }

        } else {
          this->error = true;
        }
        this->inp_buf_ix = 0;
        this->overflow = false;
        if (!error) {
          // reset ms since last only if ok
          ctx.ms_since_last_input = 0;
        }
        received = true;
      } else {
        // add to buffer
        if (inp_buf_ix < (BUF_LEN - 1)) {
          this->input_buf[inp_buf_ix++] = data;
        } else {
          // overflow :(
          this->inp_buf_ix = 0;
          this->overflow = true;
        }
      }
    }
    return received;
  }

  void write_msg() {
    StaticJsonDocument<output_msg_capacity> output_doc;
    output_doc["target_ticks_per_s"] = ctx.target_ticks_per_s;
    output_doc["target_steer_deg"] = ctx.steer_input_deg;
    output_doc["ticks_per_s"] = ctx.ticks_per_s;
    output_doc["motor_input_us"] = ctx.motor_input_us;
    output_doc["ms_since_last_input"] = static_cast<long>(ctx.ms_since_last_input);
    if (error) {
      output_doc["error"] = 1;
    }
    size_t written = serializeJson(output_doc, this->output_buf, BUF_LEN-2);
    output_buf[written] = '\n';
    output_buf[written+1] = '\0';
    Serial.write(output_buf, written+1);
  }

  void run() {
    bool received = this->read_msg();
    if (received) {
      this->write_msg();
    }
  }

} comms_task;


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

  void run() {
    if (this->elapsed_ms < ENCODER_SAMPLING_PERIOD_MS) {
      return;
    }

    int32_t ticks = this->encoder.readAndReset();
    int32_t ticks_per_s = -(ticks*1000.0)/static_cast<double>(this->elapsed_ms);

    // update global ctx
    ctx.ticks_per_s = (ticks_per_s + this->last_ticks_per_s)/2;
    //ctx.ticks_per_s = ticks_per_s;

    this->last_ticks_per_s = ticks_per_s;
    elapsed_ms = 0;
  }

} encoder_task;


struct MotorControlTask {
  // note for esc we use microseconds, not degrees
  //static constexpr int SERVO_MIN = 90-30;
  //static constexpr int SERVO_MAX = 90+30;
  static constexpr int SERVO_CENTER = 1500;
  static constexpr int SERVO_MIN = SERVO_CENTER-200;
  static constexpr int SERVO_MAX = SERVO_CENTER+200;

  static constexpr int SERVO_MOTOR_PIN = 4;

  //PWMServo servo;
  Servo servo;
  TimedPID motor_pid;
  elapsedMillis elapsed_ms;

  MotorControlTask() : motor_pid(DEFAULT_KP, DEFAULT_KI, DEFAULT_KD) { }

  void setup() {
    pinMode(SERVO_MOTOR_PIN, OUTPUT);
    // default min, max is 544, 2400
    //this->servo.attach(SERVO_MOTOR_PIN, 1000, 2000);
    this->servo.attach(SERVO_MOTOR_PIN);
    this->servo.writeMicroseconds(SERVO_CENTER);

    // this actually defines servo range so it's weird. seems like esc is 1000-2000ms, 1500 middle.
    // but default of lib is different
    // note that this is delta speed, so accel
    //this->motor_pid.setCmdRange(-10, 10);
    // fairly fast speed is 10-20 ticks each 40 ms -> 250-500 ticks/s
    // 140 ticks/s is also a fairly slow but not too slow pace

    // here the idea is max change per computation is 1 servo degree, but this is rather slow?
    // recall this is norm range, -1, 1
    // for -+ 30 degrees (60 degree range), this is 0.03333
    // double min_delta = 2.0/(SERVO_MAX - SERVO_MIN); 
    double min_delta = 2.0;
    this->motor_pid.setCmdRange(-min_delta, min_delta);
    this->elapsed_ms = 0;
  }

  void run() {
    if (ctx.pid_gains_reset) {
      this->motor_pid.setGains(ctx.kp, ctx.ki, ctx.kd);
      this->motor_pid.reset();
      ctx.pid_gains_reset = false;
      return;
    }

    if (ctx.ms_since_last_input > CMD_TIMEOUT_MS) {
      // stop
      this->servo.writeMicroseconds(SERVO_CENTER);
      return;
    }

    if (this->elapsed_ms < MOTOR_PERIOD_MS) {
        return;
    }

    ctx.ticks_error = ctx.target_ticks_per_s - ctx.ticks_per_s;

    double norm_delta = motor_pid.getCmdAutoStep(ctx.target_ticks_per_s, ctx.ticks_per_s);
    // unorthodox but fuck it
    ctx.norm_motor_input = constrain(ctx.norm_motor_input + norm_delta, -1.0, 1.0);

    double out = map(ctx.norm_motor_input, -1.0, 1.0, static_cast<double>(SERVO_MIN), static_cast<double>(SERVO_MAX));
    ctx.motor_input_us = static_cast<int>(out);
    this->servo.writeMicroseconds(static_cast<int>(out));
    //this->servo.writeMicroseconds(SERVO_CENTER);
    this->elapsed_ms = 0;
  }

} motor_control_task;


struct SteerControlTask {
  static constexpr int SERVO_CENTER = 90;
  static constexpr int SERVO_MIN = SERVO_CENTER-28;
  static constexpr int SERVO_MAX = SERVO_CENTER+28;
  static constexpr int SERVO_STEER_PIN = 3;
  Servo servo;
  elapsedMillis elapsed_ms;

  void setup() {
    pinMode(SERVO_STEER_PIN, OUTPUT);
    this->servo.attach(SERVO_STEER_PIN);
    this->elapsed_ms = 0;
  }

  void run() {
    if (ctx.ms_since_last_input > CMD_TIMEOUT_MS) {
      // stop
      this->servo.write(SERVO_CENTER);
      return;
    }

    if (this->elapsed_ms < STEER_PERIOD_MS) {
        return;
    }

    double inp = constrain(ctx.steer_input_deg, SERVO_MIN, SERVO_MAX);
    this->servo.write(inp);
    this->elapsed_ms = 0;
  }

} steer_control_task;


struct PrintTask {
  elapsedMillis elapsed_ms;

  void setup() {
    Serial.begin(115200);
  }

  void run() {
    if (elapsed_ms >= 100) {
      Serial.print(ctx.ticks_per_s);
      Serial.print("\n");
      elapsed_ms = 0;
    }
  }
} print_task;


void setup() {
  encoder_task.setup();
  motor_control_task.setup();
  steer_control_task.setup();
  comms_task.setup();
  //print_task.setup();
}


void loop() {
  encoder_task.run();
  motor_control_task.run();
  steer_control_task.run();
  comms_task.run();
  //print_task.run();
}

//vim: filetype=cpp:
