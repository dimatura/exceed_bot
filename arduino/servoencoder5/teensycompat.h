#ifndef SERVOCOMPAT_H_BXUA1V2R
#define SERVOCOMPAT_H_BXUA1V2R

#if ON_TEENSY
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <Servo.h>
#else
#include <elapsedMillis.h>
struct Servo {
    void attach(int pin) {
    }

    void write(int degrees) {
    }

    void writeMicroseconds(int us) {
    }
};

struct Encoder {
    Encoder(int pina, int pinb) {
    }

    int32_t read() {
      return 0;
    }

    int32_t readAndReset() {
      return 0;
    }
};
#endif

#endif /* end of include guard: SERVOCOMPAT_H_BXUA1V2R */
