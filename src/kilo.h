#include "ir/IRremote.h"
#include "rgbUltrasonic/RgbUltrasonic.h"

typedef enum {
  RESTING,
  MOVING_FORWARD
} state;

typedef enum {
  NONE,
  FORWARD,
  STAND
} remote_key;

decode_results remote_output;

RgbUltrasonic ultrasonic(6, 7);
IRrecv irrecv(IR_PIN);

void kilo_loop() {

  state current = RESTING;
  int targetFrame[DOF];

  switch remote() {
  case FORWARD:
    ultrasonic.SetRgbEffect(E_RGB_ALL, RGB_RED)
  case STAND:
    ultrasonic.SetRgbEffect(E_RGB_ALL, RGB_GREEN)
  }
}

float distance() {
  return ultrasonic.GetUltrasonicDistance();
}

remote_key remote() {
  if (irrecv.decode(&results)) {
    switch (results.value) {
    case 0xFF629D:
      return FORWARD;
    case 0xFF02FD:
      return STAND;
    }
  } else {
    return NONE;
  }
}
