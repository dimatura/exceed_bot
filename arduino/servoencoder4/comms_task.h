#ifndef COMMS_TASK_H_V7T8PQYE
#define COMMS_TASK_H_V7T8PQYE

#include <ArduinoJson.h>
#include "constants.h"
#include "context.h"


#define STEER_INPUT_DEG_KEY "steer_input_deg"
#define MOTOR_INPUT_US_KEY "motor_input_us"
#define TICKS_PER_S_KEY "ticks_per_s"
#define TARGET_TICKS_PER_S_KEY "target_ticks_per_s"
#define MS_SINCE_LAST_INPUT_KEY "ms_since_last_input"
#define ERROR_KEY "error"

#define CMD_TYPE_KEY "cmd"

#define MOTION_CMD_ID "motion"
#define PID_CMD_ID "pid"
#define PING_CMD_ID "ping"


struct CommsTask {
  static constexpr size_t BUF_LEN = 1024;
  static constexpr size_t MAX_CMD_LENGTH = 20;
  uint8_t input_buf[BUF_LEN];
  uint8_t output_buf[BUF_LEN];
  size_t inp_buf_ix = 0;

  // overkill sizes. this would be around 300 bytes
  static const int input_msg_capacity = JSON_ARRAY_SIZE(2) + JSON_OBJECT_SIZE(8) + 160;
  static const int output_msg_capacity = JSON_OBJECT_SIZE(8) + 160;

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

  bool handle_motor_cmd(JsonObject& input_doc, GlobalContext* ctx) {
    JsonVariant tticks_var = input_doc[TARGET_TICKS_PER_S_KEY];
    JsonVariant steer_input_deg_var = input_doc[STEER_INPUT_DEG_KEY];

    // set at least one of these
    if (tticks_var.isNull() && steer_input_deg_var.isNull()) {
      return false;
    }

    if (!tticks_var.isNull()) {
      ctx->target_ticks_per_s = tticks_var.as<float>();
    }
    if (!steer_input_deg_var.isNull()) {
      ctx->steer_input_deg = steer_input_deg_var.as<int>();
    }

    return true;
  }

  bool handle_pid_cmd(JsonObject& input_doc, GlobalContext* ctx) {
    JsonVariant kp = input_doc["kp"];
    JsonVariant ki = input_doc["ki"];
    JsonVariant kd = input_doc["kd"];
    //TODO: check set at least one

    if (!kp.isNull()) { ctx->kp = kp.as<float>(); ctx->pid_gains_reset = true; }
    if (!ki.isNull()) { ctx->ki = ki.as<float>(); ctx->pid_gains_reset = true; }
    if (!kd.isNull()) { ctx->kd = kd.as<float>(); ctx->pid_gains_reset = true; }
    this->reply_pid_cmd(ctx);
    return true;
  }

  bool handle_ping_cmd(GlobalContext *ctx) {
    this->reply_ping_cmd(ctx);
    return true;
  }

  bool parse_cmd(GlobalContext* ctx) {
    StaticJsonDocument<input_msg_capacity> input_doc;
    // decode
    DeserializationError err = deserializeJson(input_doc, this->input_buf, BUF_LEN-1);
    if (err != DeserializationError::Ok) {
      return false;
    }

    JsonArray docArr = input_doc.as<JsonArray>();
    if (docArr.isNull() || docArr.size() != 2) {
      return false;
    }

    //JsonVariant cmd_var = input_doc[CMD_TYPE_KEY];
    JsonVariant cmd_var = docArr[0];
    JsonVariant params_var = docArr[1];
    if (cmd_var.isNull() || params_var.isNull()) {
      return false;
    }
    const char * cmd = cmd_var.as<const char*>();
    JsonObject params = params_var.as<JsonObject>();
    if (params.isNull()) {
      return false;
    }
    if (strncmp(cmd, MOTION_CMD_ID, MAX_CMD_LENGTH) == 0) {
      return this->handle_motor_cmd(params, ctx);
    } else if (strncmp(cmd, PID_CMD_ID, MAX_CMD_LENGTH) == 0) {
      return this->handle_pid_cmd(params, ctx);
    } else if (strncmp(cmd, PING_CMD_ID, MAX_CMD_LENGTH) == 0) {
      return this->handle_ping_cmd(ctx);
    } else {
      // assume one of x commands
      return false;
    }
    // shouldn't reach
    return true;
  }


  void run(GlobalContext* ctx) {
    //int buf_read = Serial.readBytesUntil('\n', input_buf, BUF_LEN - 1);
    while (Serial.available() > 0) {
      uint8_t data = Serial.read();
      if (data == '\n') {
        this->error = !this->parse_cmd(ctx);
        this->inp_buf_ix = 0;
        this->overflow = false;
        if (!this->error) {
          // reset ms since last only if ok
          ctx->ms_since_last_input = 0;
        } else {
          this->reply_error(ctx);
        }
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
  }

  // note: Serial.write: if there is space in buffer, returns immediately
  // else, will block until buffer has free space
  // TODO: check for availableForWrite?
  void reply_motor_cmd(GlobalContext* ctx) {
    StaticJsonDocument<output_msg_capacity> output_doc;
    output_doc[TARGET_TICKS_PER_S_KEY] = ctx->target_ticks_per_s;
    output_doc[STEER_INPUT_DEG_KEY] = ctx->steer_input_deg;
    output_doc[TICKS_PER_S_KEY] = ctx->ticks_per_s;
    output_doc[MOTOR_INPUT_US_KEY] = ctx->motor_input_us;
    output_doc[MS_SINCE_LAST_INPUT_KEY] = static_cast<long>(ctx->ms_since_last_input);
    if (!Serial.availableForWrite()) {
      return;
    }
    size_t written = serializeJson(output_doc, this->output_buf, BUF_LEN-2);
    output_buf[written] = '\n';
    output_buf[written+1] = '\0';
    Serial.write(output_buf, written+1);
  }

  void reply_pid_cmd(GlobalContext* ctx) {
    StaticJsonDocument<output_msg_capacity> output_doc;
    output_doc["kp"] = ctx->kp;
    output_doc["ki"] = ctx->ki;
    output_doc["kd"] = ctx->kd;
    output_doc[MS_SINCE_LAST_INPUT_KEY] = static_cast<long>(ctx->ms_since_last_input);
    if (!Serial.availableForWrite()) {
      return;
    }
    size_t written = serializeJson(output_doc, this->output_buf, BUF_LEN-2);
    output_buf[written] = '\n';
    output_buf[written+1] = '\0';
    Serial.write(output_buf, written+1);
  }

  void reply_error(GlobalContext* ctx) {
    StaticJsonDocument<output_msg_capacity> output_doc;
    output_doc[ERROR_KEY] = 1;
    output_doc[MS_SINCE_LAST_INPUT_KEY] = static_cast<long>(ctx->ms_since_last_input);
    if (!Serial.availableForWrite()) {
      return;
    }
    size_t written = serializeJson(output_doc, this->output_buf, BUF_LEN-2);
    output_buf[written] = '\n';
    output_buf[written+1] = '\0';
    Serial.write(output_buf, written+1);
  }

  void reply_ping_cmd(GlobalContext* ctx) {
    StaticJsonDocument<output_msg_capacity> output_doc;
    output_doc["pong"] = 1;
    output_doc[MS_SINCE_LAST_INPUT_KEY] = static_cast<long>(ctx->ms_since_last_input);
    if (!Serial.availableForWrite()) {
      return;
    }
    size_t written = serializeJson(output_doc, this->output_buf, BUF_LEN-2);
    output_buf[written] = '\n';
    output_buf[written+1] = '\0';
    Serial.write(output_buf, written+1);
  }

};

#endif /* end of include guard: COMMS_TASK_H_V7T8PQYE */
