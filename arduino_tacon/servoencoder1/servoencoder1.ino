#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <PWMServo.h>
#include <TimedPID.h>
#include <ArduinoJson.h>
//#include <PacketSerial.h>


static constexpr int LED_PIN = 13;
//static constexpr int SERVO2_PIN = 5;


struct GlobalContext {
  double ticks_per_s = 0.0;
  double target_ticks_per_s = 0.0;
  double ticks_error = 0.0;
  int steer_input_deg = 90;
  int motor_input_deg = 90;
  elapsedMillis ms_since_last_input;
} ctx;


struct InputMsg {
  double target_ticks_per_s = 0.0;
  int steer_input_deg = 90;
} input_msg;


struct CommsTask {
  static constexpr size_t BUF_LEN = 256;
  uint8_t input_buf[BUF_LEN];
  uint8_t output_buf[BUF_LEN];
  size_t inp_buf_ix = 0;

  static const int input_msg_capacity = JSON_OBJECT_SIZE(2);
  static const int output_msg_capacity = JSON_OBJECT_SIZE(4) + 60;
  StaticJsonDocument<input_msg_capacity> input_doc;
  StaticJsonDocument<output_msg_capacity> output_doc;

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
    bool received = false;
    while (Serial.available() > 0) {
      uint8_t data = Serial.read();
      if (data == '\n') {
        // reset ms since last
        ctx.ms_since_last_input = 0;
        // decode
        DeserializationError err = deserializeJson(this->input_doc, this->input_buf, BUF_LEN-1);
        this->error = false;
        if (err == DeserializationError::Ok) {
          JsonVariant tticks_var = this->input_doc["tticks"];
          JsonVariant steer_input_deg_var = this->input_doc["tsteer"];
          if (!tticks_var.isNull()) {
            ctx.target_ticks_per_s = tticks_var.as<double>();
          } else {
            this->error = true;
          }

          if (!steer_input_deg_var.isNull()) {
            ctx.steer_input_deg = steer_input_deg_var.as<int>();
          } else {
            this->error = true;
          }
        } else {
          this->error = true;
        }
        this->inp_buf_ix = 0;
        this->overflow = false;
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
    this->output_doc["tticks"] = ctx.target_ticks_per_s;
    this->output_doc["tsteer"] = ctx.steer_input_deg;
    this->output_doc["ticks"] = ctx.ticks_per_s;
    this->output_doc["motor"] = ctx.motor_input_deg;
    size_t written = serializeJson(this->output_doc, this->output_buf, BUF_LEN-2);
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
  static constexpr int ENCODER_SAMPLING_PERIOD_MS = 40;
  static constexpr double SMOOTH_ALPHA = 0.8;
  // this tpe automagically increases
  elapsedMillis elapsed_ms;
  Encoder encoder;
  double ticks_per_s;
  long old_position;

  EncoderTask() :
      encoder(INPUT0_PIN, INPUT1_PIN),
      ticks_per_s(0),
      old_position(-999) { }

  void setup() {

  }

  void run() {
    if (elapsed_ms < ENCODER_SAMPLING_PERIOD_MS) {
      return;
    }

    long new_position = this->encoder.read();
    // forward is negative
    double delta_ticks = static_cast<double>(-(new_position - old_position));
    double ticks_per_ms = delta_ticks/static_cast<double>(ENCODER_SAMPLING_PERIOD_MS);
    //this->ticks_per_s = 1000.0*ticks_per_ms;
    // some smoothing
    this->ticks_per_s = (1000.0*ticks_per_ms) * SMOOTH_ALPHA + (this->ticks_per_s)*(1.0-SMOOTH_ALPHA);

    // update global ctx
    ctx.ticks_per_s = this->ticks_per_s;

    // reset
    old_position = new_position;
    elapsed_ms = 0;
  }

} encoder_task;


struct MotorControlTask {
  static constexpr int SERVO_MIN = 90-40;
  static constexpr int SERVO_MAX = 90+40;

  // zumo params
  //static constexpr double KP = 0.05;
  //static constexpr double KD = 0.0;
  //static constexpr double KI = 0.001;

  static constexpr int SERVO_MOTOR_PIN = 4;

  static constexpr double KP = 0.00005;
  static constexpr double KD = 0.0;
  static constexpr double KI = 0.000001;

  PWMServo servo;
  TimedPID motor_pid;

  MotorControlTask() : motor_pid(KP, KI, KD) { }

  void setup() {
    pinMode(SERVO_MOTOR_PIN, OUTPUT);
    this->servo.attach(SERVO_MOTOR_PIN, 1000, 2000);

    // this actually defines servo range so it's weird. seems like esc is 1000-2000ms, 1500 middle.
    // but default of lib is different
    // note that this is delta speed, so accel
    this->motor_pid.setCmdRange(-10, 10);
    // fairly fast speed is 10-20 ticks each 40 ms
  }

  void run() {
    double inp = constrain(ctx.motor_input_deg, SERVO_MIN, SERVO_MAX);
#if 0
    double target = 8;
    double error = (target - delta);
    out += motor_pid.getCmdAutoStep(target, delta);
    out = constrain(out, 90.0, 110.0);
#endif
    this->servo.write(inp);
  }

} motor_control_task;


struct SteerControlTask {
  static constexpr int SERVO_MIN = 40;
  static constexpr int SERVO_MAX = 140;
  static constexpr int SERVO_STEER_PIN = 3;
  PWMServo servo;

  void setup() {
    pinMode(SERVO_STEER_PIN, OUTPUT);
    this->servo.attach(SERVO_STEER_PIN);
  }

  void run() {
    double inp = constrain(ctx.steer_input_deg, SERVO_MIN, SERVO_MAX);
    this->servo.write(inp);
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
