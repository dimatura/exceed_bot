#ifndef CONSTANTS_H_NG5TRCGM
#define CONSTANTS_H_NG5TRCGM

#define ON_TEENSY 1
#define ON_M5 0

//static constexpr int SERVO2_PIN = 5;
//static constexpr float DEFAULT_KP = 0.00005;
//static constexpr float DEFAULT_KD = 0.0;
//static constexpr float DEFAULT_KI = 0.000001;

//static constexpr float DEFAULT_KP = 6e-4f;
static constexpr float DEFAULT_KP = 0.00f;
static constexpr float DEFAULT_KD = 0.00f;
static constexpr float DEFAULT_KI = 0.0001f;
// 0.001 is a tad too small, 0.0012 already gets the thing moving from stop if target=200
static constexpr float DEFAULT_KF = 0.0012f;

static constexpr long CMD_TIMEOUT_MS = 1800;

#endif /* end of include guard: CONSTANTS_H_NG5TRCGM */
