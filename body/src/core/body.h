// body: the logic half of the 50 Hz tick (doc §3a steps 2, 4, 5, 6) plus
// the reflexes of P0: battery, link watchdog, IMU health, warm-boot hold.
//
// Pure C++: the caller (src/app/main.cpp) does the hardware steps (IMU FIFO
// read, PCA9685 burst, ADC, ultrasonic) and passes the results in TickIn.
// That split is what lets body_tick() run, unchanged, in the host tests.
#ifndef KILO_CORE_BODY_H
#define KILO_CORE_BODY_H
#include <stdint.h>
#include "arbiter.h"
#include "attitude.h"
#include "balance.h"
#include "battery.h"
#include "config.h"
#include "events.h"
#include "notes.h"
#include "params.h"

// Durations in 20 ms ticks. The watchdog's are live params (core/params.h).
#define WD_STOP_TICKS param_ms_ticks(KILO_PARAM_WD_STOP_MS)        // 0.5 s without a valid frame: stop
#define WD_REST_TICKS param_ms_ticks(KILO_PARAM_WD_REST_MS)        // 1.5 s: lie down
#define WD_LIMP_TICKS ((uint16_t)(PARAM(WD_LIMP_S) * 50))          // 10 s after REST: limp
#define REST_MOVE_TICKS 50   // reflex moves to REST take 1 s
#define BATT_LIMP_TICKS 100  // low battery: limp 2 s after REST
#define WARM_HOLD_TICKS 50   // warm boot: hold at most 1 s waiting for the IMU
#define IMU_STALE_TICKS 10   // no fresh packet for 10 ticks -> IMU_FAULT
#define IMU_ERR_RUN 3        // or 3 consecutive I2C errors

enum ImuStatus : uint8_t { IMU_NONE = 0, IMU_FRESH = 1, IMU_ERR = 2 };

// Who holds the REFLEX owner. A higher value may take it from a lower one.
enum ReflexHolder : uint8_t { RH_NONE = 0, RH_WARM_HOLD = 1, RH_WATCHDOG = 2, RH_BATTERY = 3 };

// Goal results. The values are the ACK status codes, which EVENT
// GOAL_REFUSED reuses as its reason (core/link.cpp static_asserts them).
enum GoalResult : uint8_t {
  GOAL_OK = 0,
  GOAL_BAD_ARG = 2,
  GOAL_LOW_BATT = 7,
  GOAL_STALE = 10,
  GOAL_UNCALIBRATED = 11,  // STAND refused while CONFIG_DEFAULTED (doc §3b)
};

struct TickIn {
  uint8_t imu;          // ImuStatus of this tick's FIFO read
  Attitude att;         // valid when imu == IMU_FRESH
  uint8_t batt_valid;   // a new ADC sample (10 Hz)
  uint16_t batt_counts;
};

struct LinkWatchdog {
  uint8_t armed;  // a valid frame has been seen since boot
  uint8_t stage;  // 0 ok, 1 stopped, 2 resting, 3 limp
  uint16_t since;       // ticks since the last valid frame
  uint16_t limp_timer;  // the watchdog reflex's own LIMP timer
};

struct Body {
  Config* cfg;
  Arbiter arb;
  Battery batt;
  EventQueue ev;
  Notes notes;
  Attitude att;
  LinkWatchdog wd;
  uint16_t tick;
  uint8_t epoch;
  uint8_t holder;  // ReflexHolder
  uint16_t batt_limp_timer;
  uint8_t hold_ticks;
  uint8_t balance_req;  // SET_POSTURE flags bit 0; cleared by an epoch bump
  Balance bal;          // the balance layer: SET_BALANCE settings + controller
  // IMU health
  uint8_t imu_fault;
  uint8_t imu_stale_run, imu_err_run;
  uint16_t imu_stale_total;
};

// warm_angles: canonical-order angles read back from the PCA9685, or null
// for a cold start. imu_ok: did dmpInitialize succeed.
void body_init(Body& b, Config* cfg, const int16_t* warm_angles, bool imu_ok);

// Steps 2 and 4-6 of the tick. Fills all 16 PCA9685 channel counts.
void body_tick(Body& b, const TickIn& in, uint16_t counts[JOINT_COUNT]);

// ---- Integration points for the link layer (protocol round) ----
// Call on every CRC-valid frame from the Pi: feeds the link watchdog.
void body_link_frame(Body& b);
// SET_POSTURE goal.
GoalResult body_goal_posture(Body& b, uint8_t epoch, uint8_t posture, uint8_t ticks, uint8_t flags);
// SET_BALANCE goal: enable, Kp (Q8), Kd (ms), dead band and clamp (deg).
// Live, not persisted. BAD_ARG if enable > 1 or a band/clamp > 30 deg.
GoalResult body_goal_balance(Body& b, const uint8_t p[5]);
// Balance runs this tick: requested (posture flag or SET_BALANCE enable),
// eligible (doc §3c: IDLE SIT or STAND, settled, no REFLEX) and no IMU_FAULT.
// TELEMETRY flags bit 0.
inline bool body_balance_active(const Body& b) { return b.bal.active; }
// IMU re-initialised (hal finished a runtime DMP upload while LIMP).
void body_imu_recovered(Body& b);
// TELEMETRY.mode (§4): 1 LIMP, 2 IDLE, 7 LOW_BATT, 9 LINK_LOST.
uint8_t body_mode(const Body& b);

void body_epoch_bump(Body& b);

#endif
