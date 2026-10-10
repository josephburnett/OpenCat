// attitude: DMP packet -> pitch, roll and body rates, in integers (doc §3f).
//
// The DMP (the MPU6050's on-chip motion processor) fuses gyro and
// accelerometer into an orientation quaternion. We only need which way is
// "down" in the body frame (the gravity vector g), and pitch/roll from it:
//   g = (2(qx qz - qw qy), 2(qw qx + qy qz), qw^2 - qx^2 - qy^2 + qz^2)
//   pitch = atan2(gx, sqrt(gy^2 + gz^2)),  roll = atan2(gy, gz)
// plus the legacy "pitch reflection" when the body is upside down (gz < 0),
// kept so tricks, the fall classifier and the P1 A/B test against the old
// firmware all see the same numbers.
#ifndef KILO_CORE_ATTITUDE_H
#define KILO_CORE_ATTITUDE_H
#include <stdint.h>

#define DMP_PACKET_SIZE 28

struct Attitude {
  int16_t pitch;  // a16, nose up + (legacy sign, incl. reflection)
  int16_t roll;   // a16
  int16_t rate_pitch, rate_roll, rate_yaw;  // a16/s from the raw gyro
  int16_t gz;     // gravity z, Q14 (negative = upside down), for reflexes
};

// Quaternion from the packet: four big-endian int32 Q30 at bytes 0, 4, 8, 12
// (w, x, y, z). Like legacy dmpGetQuaternion(int16_t*), keep the high 16
// bits, which is Q14.
void attitude_quat_q14(const uint8_t* pkt, int16_t q[4]);

// Pitch and roll from a Q14 quaternion. Exposed for the host test.
// Returns false (and leaves out untouched) if q is not near unit length.
bool attitude_from_quat(const int16_t q[4], Attitude& out);

// Full decode of one 28 B packet: quaternion -> pitch/roll, and the raw
// gyro (bytes 22..27, big-endian int16, +-2000 deg/s full scale) -> rates.
bool attitude_from_packet(const uint8_t* pkt, Attitude& out);

#endif
