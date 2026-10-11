// reflex: the fall reflex's pure parts (doc §3g "Fall"). P1.
//
// The detector and the attitude tests are pure functions of the attitude,
// host-tested on their own; body.cpp runs the state machine around them
// (owner, epoch, events, the `rc` sequence: core/posture.h POSTURE_RC):
//
//   UPRIGHT -> FALLEN   |roll| or |pitch| > FALL_DEG (75 deg) on FALL_SAMPLES
//                       (6) consecutive fresh IMU samples (120 ms at 50 Hz);
//                       one sample never trips it (P0 finding 8: a handled
//                       sample read -70 deg). Epoch bump (a playing trick
//                       ends with TRICK_DONE FELL), REFLEX holds the pose,
//                       EVENT FELL (+-1 roll / +-2 pitch, the angle's sign),
//                       a chirp. Then waits until still: every body rate
//                       < 20 deg/s on FALL_STILL_TICKS (10) samples.
//   FALLEN  -> RC       roll fall: play `rc`, mirrored for roll > 0 and
//                       flipped on every other try, up to FALL_TRIES (3).
//                       Pitch fall: FAILED.
//   RC      -> CHECK    after rc's last frame: hold it for up to
//                       FALL_CHECK_TICKS (1 s), waiting for upright.
//   CHECK   -> FALLEN   not upright after 1 s: the next try (still first),
//                       or FAILED after the third.
//   CHECK   -> UPRIGHT  |roll|, |pitch| < 20 deg on FALL_LEVEL_TICKS (25)
//                       consecutive samples: IDLE STAND (REST if the config
//                       is DEFAULTED), the joints hand back with the usual
//                       XFADE_TICKS crossfade, EVENT RECOVERED. (Righted by
//                       hand while FALLEN: once still, rc plays from
//                       upright, ends standing, and CHECK passes.)
//   FAILED              REST, EVENT RECOVERY_FAILED, LIMP after
//                       FALL_LIMP_TICKS (2 s), then the reflex lets go.
//
// Detection is off in LIMP and CALIB, under IMU_FAULT, during FALL_OK
// tricks and while another reflex holds the joints (warm-boot hold,
// watchdog REST, low battery). An IMU_FAULT after the trip is FAILED; a
// more important reflex taking the joints ends the fall reflex silently.
// Goals while it holds the joints: SET_POSTURE LIMP ends it (the Pi's
// emergency stop); every other motion goal is refused FALLEN.
#ifndef KILO_CORE_REFLEX_H
#define KILO_CORE_REFLEX_H
#include <stdint.h>
#include "attitude.h"

#define FALL_DEG 75           // tilt threshold, deg
#define FALL_SAMPLES 6        // consecutive samples over it (120 ms)
#define FALL_STILL_DPS 20     // "still": every body rate below this
#define FALL_STILL_TICKS 10
#define FALL_LEVEL_DEG 20     // "upright": |roll|, |pitch| below this
#define FALL_LEVEL_TICKS 25   // for 0.5 s
#define FALL_TRIES 3          // rc attempts
#define FALL_CHECK_TICKS 50   // after rc, wait this long for upright (tries 1 s apart)
#define FALL_LIMP_TICKS 100   // FAILED: REST, then LIMP 2 s later

enum FallState : uint8_t { FS_UPRIGHT = 0, FS_FALLEN = 1, FS_RC = 2, FS_CHECK = 3, FS_FAILED = 4 };

struct Fall {
  uint8_t state;   // FallState
  uint8_t over;    // consecutive samples over FALL_DEG (UPRIGHT)
  uint8_t still;   // consecutive still samples (FALLEN)
  uint8_t level;   // consecutive upright samples (after the trip)
  uint8_t tries;   // rc attempts started
  uint8_t frame;   // rc frame playing (RC)
  uint8_t mirror;  // POSTURE_RC_MIRROR or 0, for this try
  int8_t axis;     // the FELL arg: +-1 roll, +-2 pitch
  uint16_t timer;  // ticks left in the rc frame, CHECK, or FAILED's LIMP timer
};

// One fresh sample through the detector. Returns 0, or the FELL arg once
// FALL_SAMPLES consecutive samples were over FALL_DEG: a pitch fall (+-2)
// when the body's long axis is within 15 deg of vertical (|pitch| folded
// about 90 deg, since the legacy reflection reports pitch past 90 deg
// upside down), else a roll fall (+-1, the sign of roll: on a side or on
// the back). A sample under the threshold restarts the count.
int8_t fall_detect(uint8_t& over, const Attitude& a);
// |roll| and |pitch| under FALL_LEVEL_DEG.
bool fall_level(const Attitude& a);
// Every body rate under FALL_STILL_DPS.
bool fall_still(const Attitude& a);

#endif
