#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <PWMServo.h>
#include <TimedPID.h>
#include <ArduinoJson.h>
#include <CircularBuffer.h>
//#include <PacketSerial.h>

static constexpr int LED_PIN = 13;
//static constexpr int SERVO2_PIN = 5;

//static constexpr double DEFAULT_KP = 0.00005;
//static constexpr double DEFAULT_KD = 0.0;
//static constexpr double DEFAULT_KI = 0.000001;

static constexpr double DEFAULT_KP = 0.01;
static constexpr double DEFAULT_KD = 0.00;
static constexpr double DEFAULT_KI = 0.00;

static constexpr long CMD_TIMEOUT_MS = 4000;

#define STEER_INPUT_DEG_KEY "steer_input_deg"
#define MOTOR_INPUT_DEG_KEY "motor_input_deg"
#define TICKS_PER_S_KEY "ticks_per_s"
#define TARGET_TICKS_PER_S_KEY "target_ticks_per_s"

#define CMD_TYPE_KEY "cmd"
#define MOTION_CMD_KEY 'm'
#define PID_CMD_KEY 'p'

struct GlobalContext {
  double ticks_per_s = 0.0;
  double target_ticks_per_s = 0.0;
  double ticks_error = 0.0;
  int steer_input_deg = 90;
  int motor_input_deg = 90;

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

  static const int input_msg_capacity = JSON_OBJECT_SIZE(3) + 80;
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

  void parse_motor_cmd(StaticJsonDocument<input_msg_capacity>& input_doc) {
      JsonVariant tticks_var = input_doc[TARGET_TICKS_PER_S_KEY];
      JsonVariant steer_input_deg_var = input_doc[STEER_INPUT_DEG_KEY];

      if (!tticks_var.isNull()) { ctx.target_ticks_per_s = tticks_var.as<double>(); } else { this->error = true; }
      if (!steer_input_deg_var.isNull()) { ctx.steer_input_deg = steer_input_deg_var.as<int>(); } else { this->error = true; }
  }

  void parse_pid_cmd(StaticJsonDocument<input_msg_capacity>& input_doc) {
      JsonVariant kp = input_doc["kp"];
      JsonVariant ki = input_doc["ki"];
      JsonVariant kd = input_doc["kd"];

      if (!kp.isNull()) { ctx.kp = kp.as<double>(); ctx.pid_gains_reset = true; }
      if (!ki.isNull()) { ctx.ki = ki.as<double>(); ctx.pid_gains_reset = true; }
      if (!kd.isNull()) { ctx.kd = ki.as<double>(); ctx.pid_gains_reset = true; }
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
            JsonVariant cmd_var = input_doc[CMD_TYPE_KEY];
            if (!cmd_var.isNull()) {
                char cmd = cmd_var.as<char>();
                if (cmd.as<char>() == MOTION_CMD_KEY) {
                    this->parse_motor_cmd(input_doc);
                } else if (cmd.as<char>() == PID_CMD_KEY) {
                    this->parse_pid_cmd(input_doc);
                }
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
    output_doc["motor_input_deg"] = ctx.motor_input_deg;
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

  //static constexpr int ENCODER_SAMPLING_PERIOD_MS = 60;
  //double ki = 240;
  //double kp = 18;

  static constexpr int ENCODER_SAMPLING_PERIOD_MS = 45;
  double ki = 40;
  double kp = 8;

  //static constexpr int ENCODER_SAMPLING_PERIOD_MS = 80;
  //double ki = 200;
  //double kp = 12; //16;

  //static constexpr int ENCODER_SAMPLING_PERIOD_MS = 160;
  //double ki = 24.0;
  //double kp = 6.0;

  // 40, 8 works well with velocity_integrator

  // this tpe automagically increases
  elapsedMillis elapsed_ms;
  long elapsed_ms_sum = 0;
  Encoder encoder;

  double ticks_per_s;

  double last_position = 0.0;
  double est_position = 0.0;
  double est_velocity = 0.0;
  double velocity_integrator = 0.0;

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

    int32_t new_position = this->encoder.read();
    est_position += ((est_velocity * static_cast<double>(elapsed_ms)) / 1000.0);
    double position_error = (new_position - est_position);

    velocity_integrator += ((position_error * ki * static_cast<double>(elapsed_ms))/1000.0);
    est_velocity = position_error * kp + velocity_integrator;

    //this->ticks_per_s = -(count0_sum_*1000.0)/static_cast<double>(elapsed_ms_sum);
    //this->ticks_per_s = SMOOTH_ALPHA*velocity_integrator + (1.0-SMOOTH_ALPHA)*this->ticks_per_s;
    //this->ticks_per_s = velocity_integrator;
    this->ticks_per_s = est_velocity;

    // update global ctx
    ctx.ticks_per_s = -this->ticks_per_s;
    //Serial.println("vel_int:" + String(velocity_integrator) + ", err:" + String(position_error));
    //Serial.println("est_pos:" + String(est_position) + ", pos:" + new_position);
    //Serial.println(position_error);
    // reset
    elapsed_ms = 0;
  }

} encoder_task;




struct MotorControlTask {
  static constexpr int SERVO_MIN = 90-40;
  static constexpr int SERVO_MAX = 90+40;

  static constexpr int SERVO_MOTOR_PIN = 4;

  PWMServo servo;
  TimedPID motor_pid;

  MotorControlTask() : motor_pid(DEFAULT_KP, DEFAULT_KI, DEFAULT_KD) { }

  void setup() {
    pinMode(SERVO_MOTOR_PIN, OUTPUT);
    this->servo.attach(SERVO_MOTOR_PIN, 1000, 2000);
    this->servo.write(90);

    // this actually defines servo range so it's weird. seems like esc is 1000-2000ms, 1500 middle.
    // but default of lib is different
    // note that this is delta speed, so accel
    //this->motor_pid.setCmdRange(-10, 10);
    // fairly fast speed is 10-20 ticks each 40 ms

    this->motor_pid.setCmdRange(-1.0, 1.0);
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
      this->servo.write(90);
      return;
    }

    ctx.ticks_error = ctx.target_ticks_per_s - ctx.ticks_per_s;

    double norm_out = motor_pid.getCmdAutoStep(ctx.target_ticks_per_s, ctx.ticks_per_s);
    double out = map(norm_out, -1.0, 1.0, SERVO_MIN, SERVO_MAX);
    //out = constrain(out, SERVO_MIN, SERVO_MAX);
    ctx.motor_input_deg = out;
    this->servo.write(static_cast<int>(out));
  }

} motor_control_task;


struct SteerControlTask {
  static constexpr int SERVO_MIN = 90-28;
  static constexpr int SERVO_MAX = 90+28;
  static constexpr int SERVO_STEER_PIN = 3;
  PWMServo servo;

  void setup() {
    pinMode(SERVO_STEER_PIN, OUTPUT);
    this->servo.attach(SERVO_STEER_PIN);
  }

  void run() {
    if (ctx.ms_since_last_input > CMD_TIMEOUT_MS) {
      // stop
      this->servo.write(90);
      return;
    }

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
