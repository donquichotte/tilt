#include "tilt.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "quaternion.h"

#define MS_PER_US 1000.0f
#define GRAVITY_CALIBRATION_CONSTANT 0.1f

/* tilt estimation algorithm from accelerometers and gyroscopes,
 *  taken from [1].
 *
 *
 *  [1] https://stanford.edu/class/ee267/notes/ee267_notes_imu.pdf
 */

static void printvec(const float in[3]) {
  printf("v[0] = %f\n", in[0]);
  printf("v[1] = %f\n", in[1]);
  printf("v[2] = %f\n", in[2]);
}

static void printquat(const float in[4]) {
  printf("q[0] = %f\n", in[0]);
  printf("q[1] = %f\n", in[1]);
  printf("q[2] = %f\n", in[2]);
  printf("q[3] = %f\n", in[3]);
}

static float clampFloat(const float in, const float min, const float max) {
  if (in < min)
    return min;
  if (in > max)
    return max;
  return in;
}

void tilt_init(tilt_complementaryFilter_t *tilt, float alpha) {
  tilt->q[0] = 1.0f;
  tilt->q[1] = 0.0f;
  tilt->q[2] = 0.0f;
  tilt->q[3] = 0.0f;
  tilt->alpha = alpha;

  tilt->g[0] = 0.0f;
  tilt->g[1] = 0.0f;
  tilt->g[2] = 1.0f;

  tilt->state = TILT_UNCALIBRATED;
}

void tilt_calibrate(tilt_complementaryFilter_t *tilt, const float acc[3]) {
  float alpha = GRAVITY_CALIBRATION_CONSTANT;
  tilt->g[0] = (1.0f - alpha) * tilt->g[0] + alpha * acc[0];
  tilt->g[1] = (1.0f - alpha) * tilt->g[1] + alpha * acc[1];
  tilt->g[2] = (1.0f - alpha) * tilt->g[2] + alpha * acc[2];
  vec3_normalize(tilt->g, tilt->g);
}

bool tilt_update(tilt_complementaryFilter_t *tilt, const float acc[3],
                 const float gyro[3], const float dt) {

  /*
   * integrate gyro
   */

  /* eqn 17 */
  float qdelta[4];
  float angle = dt * vec3_abs(gyro);
  float axis[3];
  float qOmega[4];

  /* if the gyro vector is small, do nothing. */
  if (vec3_normalize(gyro, axis)) {
    quaternion_fromAngleAndAxis(angle, axis, qdelta);

    /* eqn 18 */
    quaternion_multiply(tilt->q, qdelta, qOmega);
  } else {
    memcpy(qOmega, tilt->q, 4 * sizeof(float));
  }

  /*
   * calculate correction qc from accelerometer
   */

  /* eqn. 21, rotate acc from body frame to world frame.
  qaBody, qaWorld are vector quaternions, i.e.
  the scalar part is zero, see eqn. 14 */
  float qaBody[4];
  float qaWorld[4];
  quaternion_fromVector(acc, qaBody);

  /* eqn. 22, actual rotation */
  quaternion_rotate(qOmega, qaBody, qaWorld);
  quaternion_normalize(qaWorld, qaWorld);

  /* eqn. 23, generalized for g pointing in any direction */
  float normal[3];
  float phi;

  vec3_cross(&qaWorld[1], tilt->g, normal);

  float cosPhi = vec3_dot(tilt->g, &qaWorld[1]);
  phi = acos(clampFloat(cosPhi, -1.0f, 1.0f));

  vec3_normalize(normal, normal);

  /* eqn. 24 */
  float qt[4];
  quaternion_fromAngleAndAxis((1 - tilt->alpha) * phi, normal, qt);

  /*
   * fuse measurements, store
   */
  float qc[4];
  quaternion_multiply(qt, qOmega, qc);
  quaternion_normalize(qc, tilt->q);

  return true;
}

bool tilt_getPitchRoll(const tilt_complementaryFilter_t *tilt, float *pitch,
                       float *roll) {

  if (tilt->state != TILT_CALIBRATED)
    return false;

  /* see http://www.chrobotics.com/library/understanding-quaternions */
  float *q = (float *)tilt->q;

  *roll = atan2(2.0 * q[0] * q[1] + q[2] * q[3],
                q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);

  float sinPitch = 2.0 * (q[1] * q[3] - q[0] * q[2]);
  *pitch = -asin(clampFloat(sinPitch, -1.0f, 1.0f));

  return true;
}
