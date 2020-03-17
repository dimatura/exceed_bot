#ifndef COMMS_TASK_H_V7T8PQYE
#define COMMS_TASK_H_V7T8PQYE

#include <ArduinoJson.h>
#include "constants.h"
#include "context.h"

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

  bool read_msg(GlobalContext* ctx) {
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
            ctx->target_ticks_per_s = tticks_var.as<float>();
          } else {
            this->error = true;
          }

          JsonVariant steer_input_deg_var = input_doc["target_steer_deg"];
          if (!steer_input_deg_var.isNull()) {
            ctx->steer_input_deg = steer_input_deg_var.as<int>();
          } else {
            this->error = true;
          }

          // these are optional
          JsonVariant kp = input_doc["kp"];
          if (!kp.isNull()) {
            ctx->kp = kp.as<float>();
            ctx->pid_gains_reset = true;
          }
          JsonVariant ki = input_doc["ki"];
          if (!ki.isNull()) {
            ctx->ki = ki.as<float>();
            ctx->pid_gains_reset = true;
          }
          JsonVariant kd = input_doc["kd"];
          if (!kd.isNull()) {
            ctx->kd = kd.as<float>();
            ctx->pid_gains_reset = true;
          }

        } else {
          this->error = true;
        }
        this->inp_buf_ix = 0;
        this->overflow = false;
        if (!error) {
          // reset ms since last only if ok
          ctx->ms_since_last_input = 0;
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

  void write_msg(GlobalContext* ctx) {
    StaticJsonDocument<output_msg_capacity> output_doc;
    output_doc["target_ticks_per_s"] = ctx->target_ticks_per_s;
    output_doc["target_steer_deg"] = ctx->steer_input_deg;
    output_doc["ticks_per_s"] = ctx->ticks_per_s;
    output_doc["motor_input_us"] = ctx->motor_input_us;
    output_doc["ms_since_last_input"] = static_cast<long>(ctx->ms_since_last_input);
    if (error) {
      output_doc["error"] = 1;
    }
    size_t written = serializeJson(output_doc, this->output_buf, BUF_LEN-2);
    output_buf[written] = '\n';
    output_buf[written+1] = '\0';
    Serial.write(output_buf, written+1);
  }

  void run(GlobalContext* ctx) {
    bool received = this->read_msg(ctx);
    if (received) {
      this->write_msg(ctx);
    }
  }

};



#endif /* end of include guard: COMMS_TASK_H_V7T8PQYE */
