#ifndef TILT_H
#define TILT_H
#include <stdbool.h>

typedef enum tiltState_e {
  TILT_UNCALIBRATED = 0,
  TILT_CALIBRATED = 1
} tiltState_t;

typedef struct tilt_complementaryFilter_s {
  float q[4];  /* attitude quaternion */
  float alpha; /* filter mixing parameter, in [0,1]. 0 is acc only, 1 is
                  gyro only */
  float g[3];  /* gravity vector, set to {0, 0, 1} in tilt_init(), can be
                  calibrated */
  tiltState_t state;
} tilt_complementaryFilter_t;

void tilt_calibrate(tilt_complementaryFilter_t *tilt, const float acc[3]);

void tilt_init(tilt_complementaryFilter_t *tilt, float alpha);

bool tilt_update(tilt_complementaryFilter_t *tilt, const float acc[3],
                 const float gyro[3], const float dt);

bool tilt_getPitchRoll(const tilt_complementaryFilter_t *tilt, float *pitch,
                       float *roll);

#endif /* TILT_H */
