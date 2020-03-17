#ifndef CONSTANTS_H_NG5TRCGM
#define CONSTANTS_H_NG5TRCGM

#define ON_TEENSY 0
#define ON_M5 1

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

#endif /* end of include guard: CONSTANTS_H_NG5TRCGM */
