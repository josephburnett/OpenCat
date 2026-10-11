// body: the logic half of the 50 Hz tick (doc §3a steps 2, 4, 5, 6) plus
// the reflexes: battery, link watchdog, IMU health, warm-boot hold (P0) and
// the fall reflex (P1; core/reflex.h, fall_step in body.cpp).
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
#include "gait.h"
#include "keyframe.h"
#include "notes.h"
#include "params.h"
#include "reflex.h"

// Durations in 20 ms ticks. The watchdog's are live params (core/params.h).
#define WD_STOP_TICKS param_ms_ticks(KILO_PARAM_WD_STOP_MS)        // 0.5 s without a valid frame: stop
#define WD_REST_TICKS param_ms_ticks(KILO_PARAM_WD_REST_MS)        // 1.5 s: lie down
#define WD_LIMP_TICKS ((uint16_t)(PARAM(WD_LIMP_S) * 50))          // 10 s after REST: limp
#define REST_MOVE_TICKS 50   // reflex moves to REST take 1 s
#define BATT_LIMP_TICKS 100  // low battery: limp 2 s after REST
#define WARM_HOLD_TICKS 50   // warm boot: hold at most 1 s waiting for the IMU
#define IMU_STALE_TICKS 10   // no fresh packet for 10 ticks -> IMU_FAULT
#define IMU_ERR_RUN 3        // or 3 consecutive I2C errors
#define HEAD_HOLD_TICKS (KILO_HEAD_HOLD_MS / 20u)  // SET_HEAD hold without a refresh: 2 s

enum ImuStatus : uint8_t { IMU_NONE = 0, IMU_FRESH = 1, IMU_ERR = 2 };

// Who holds the REFLEX owner. A higher value may take it from a lower one.
enum ReflexHolder : uint8_t { RH_NONE = 0, RH_WARM_HOLD = 1, RH_FALL = 2, RH_WATCHDOG = 3, RH_BATTERY = 4 };

// Goal results. The values are the ACK status codes, which EVENT
// GOAL_REFUSED reuses as its reason (core/link.cpp static_asserts them).
enum GoalResult : uint8_t {
  GOAL_OK = 0,
  GOAL_BAD_ARG = 2,
  GOAL_BUSY = 3,
  GOAL_NO_TRICK = 6,
  GOAL_LOW_BATT = 7,
  GOAL_FALLEN = 8,  // the fall reflex holds the joints (ACK REFUSED_FALLEN)
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
  Keyframe kf;          // the trick store and player (TRICK owner)
  Fall fall;            // the fall reflex (core/reflex.h)
  Gait gait;            // the gait generator, STANCE and the body pose (core/gait.h; P2)
  // IMU health
  uint8_t imu_fault;
  uint8_t imu_stale_run, imu_err_run;
  uint16_t imu_stale_total;
};

// warm_angles: canonical-order angles read back from the PCA9685, or null
// for a cold start. imu_ok: did dmpInitialize succeed.
void body_init(Body& b, Config* cfg, const int16_t* warm_angles, bool imu_ok, uint8_t epoch = 0);

// Steps 2 and 4-6 of the tick. Fills all 16 PCA9685 channel counts.
void body_tick(Body& b, const TickIn& in, uint16_t counts[JOINT_COUNT]);

// ---- Integration points for the link layer (protocol round) ----
// Call on every CRC-valid frame from the Pi: feeds the link watchdog.
void body_link_frame(Body& b);
// SET_POSTURE goal. Any posture but STANCE also stops the gait (LIMP at
// once, the others with the usual ramp; IDLE moves meanwhile, and the legs
// crossfade to it when the gait hands them back).
GoalResult body_goal_posture(Body& b, uint8_t epoch, uint8_t posture, uint8_t ticks, uint8_t flags);
// SET_HEAD goal (payload: epoch, mode, pan, tilt, tail, max_speed_dps; doc
// §3c layer 1). mode 0 releases (never stale); mode 1 holds: STALE, FALLEN,
// LOW_BATT; BAD_ARG for mode > 1. The safety stage clamps the targets;
// -32768 leaves that joint to its owner.
GoalResult body_goal_head(Body& b, const uint8_t* p);
// SET_GAIT goal (payload: epoch, run, pattern, period_ms, duty, stride_mm,
// turn_mm, step_h_mm, body_h_mm, flags; doc §3d). BAD_ARG for a field out
// of the spec's range (0 = the pattern's default for period, duty, step and
// body height; unknown flags). run 0 stops the gait (ramp, then STANCE,
// EVENT GAIT_STOPPED) and is never stale. run 1: STALE, FALLEN, LOW_BATT,
// UNCALIBRATED (as STAND); then latched: IDLE moves to STANCE first if it is
// elsewhere, and the gait starts once STANCE has settled and no trick or
// reflex holds the legs (and restarts after one, while the goal stands).
// Re-sending the running goal only updates its parameters (slewed); a new
// pattern is a stop then a start.
GoalResult body_goal_gait(Body& b, const uint8_t* p);
// SET_BODY_POSE goal (payload: epoch, pitch, roll (a16), dheight_mm,
// xshift_mm): STALE, FALLEN, LOW_BATT. Applies to the gait and to IDLE
// STANCE (slewed; pitch and roll clamped to +-30 deg); zeroed by an epoch bump.
GoalResult body_goal_pose(Body& b, const uint8_t* p);
// The gait owns the legs (running or stopping): TELEMETRY flags GAIT.
inline bool body_gait_running(const Body& b) { return b.gait.state != GS_OFF; }
// The fall reflex holds the joints (FALLEN, RECOVERING or FAILED).
inline bool body_fallen(const Body& b) { return b.fall.state != FS_UPRIGHT; }
// SET_BALANCE goal: enable, Kp (Q8), Kd (ms), dead band and clamp (deg).
// Live, not persisted. BAD_ARG if enable > 1 or a band/clamp > 30 deg.
GoalResult body_goal_balance(Body& b, const uint8_t p[5]);
// Balance runs this tick: requested (posture flag or SET_BALANCE enable),
// eligible (doc §3c: IDLE SIT or STAND, settled, no REFLEX) and no IMU_FAULT.
// TELEMETRY flags bit 0.
inline bool body_balance_active(const Body& b) { return b.bal.active; }
// TRICK_PLAY (payload: epoch, trick_id, speed_q6, flags). Returns the ACK
// status: STALE, NO_TRICK, BAD_ARG, BUSY (a trick plays), REFUSED_FALLEN
// (the fall reflex holds the joints), LOW_BATT, BUSY (another reflex holds
// the joints, or the IDLE posture is LIMP or CALIB), UNCALIBRATED.
uint8_t body_trick_play(Body& b, const uint8_t p[4]);
// TRICK_STOP: crossfade to the end posture over `ticks` (0 = XFADE_TICKS),
// EVENT TRICK_DONE(STOPPED). Always OK (nothing playing is not an error).
void body_trick_stop(Body& b, uint8_t ticks);
// End the playing trick (if any) for `reason` (KfEnd): EVENT TRICK_DONE,
// IDLE adopts the end posture, the joints crossfade back over `ticks`
// (0 = XFADE_TICKS). (A fall ends it through body_epoch_bump(b, KF_END_FELL).)
void body_trick_end(Body& b, uint8_t reason, uint8_t ticks);
// Fall detection must stay off: a FALL_OK trick is playing (for the fall
// reflex, doc §3g).
inline bool body_fall_suppressed(const Body& b) { return kf_fall_ok(b.kf); }
// IMU re-initialised (hal finished a runtime DMP upload while LIMP).
void body_imu_recovered(Body& b);
// TELEMETRY.mode (§4): 1 LIMP, 2 IDLE, 3 GAIT (running, stopping, or
// waiting to start), 4 TRICK, 5 FALLEN, 6 RECOVERING, 7 LOW_BATT, 9 LINK_LOST.
uint8_t body_mode(const Body& b);

// A new goal epoch (doc §3c): clears the latched motion goals and ends a
// playing trick with TRICK_DONE(trick_reason) carrying the new epoch. The
// fall reflex passes KF_END_FELL; everything else is a preemption.
void body_epoch_bump(Body& b, uint8_t trick_reason = KF_END_PREEMPTED);

#endif
