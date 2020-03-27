#include "tilt.h"
#include <stdio.h>

int main(int argc, char const *argv[]) {
  tilt_complementaryFilter_t tilt;
  tilt_init(&tilt, 0.5f);

  /* calibrate gravity vector */
  for (int i = 0; i < 100; i++) {
    float acc[3] = {0.0f, 0.0f, 1.0f};
    tilt_calibrate(&tilt, acc);
  }
  tilt.state = TILT_CALIBRATED;

  /* estimate tilt */
  for (int i = 0; i < 200; i++) {
    float acc[3] = {0.00f, 0.0f, 1.0f};
    float gyro[3] = {-0.0f, 0.00f, 0.00f};
    float dt = 1.0f / 200;

    tilt_update(&tilt, acc, gyro, dt);

    float pitch;
    float roll;
    tilt_getPitchRoll(&tilt, &pitch, &roll);

    // printf("Step %d, pitch: %f, roll: %f\n", i, pitch, roll);
    printf("{\"t\": %f, \"tilt\": [%.5f, %.5f],", i * dt, pitch, roll);
    printf("\"quaternion\": [%.5f, %.5f, %.5f, %.5f]}\n", tilt.q[0], tilt.q[1],
           tilt.q[2], tilt.q[3]);
  }
  return 0;
}
