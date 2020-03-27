#ifndef QUATERNION_H
#define QUATERNION_H

#include <stdbool.h>

float vec3_abs(const float in[3]);

bool vec3_normalize(const float in[3], float out[3]);

void vec3_cross(const float u[3], const float v[3], float out[3]);

float vec3_dot(const float u[3], const float v[3]);

void quaternion_fromAngleAndAxis(const float angle, const float axis[3],
                                 float out[4]);

void quaternion_multiply(const float q[4], const float p[4], float out[4]);

void quaternion_fromVector(const float vec[3], float out[4]);

void quaternion_rotate(const float qr[4], const float qu[4], float out[4]);

void quaternion_normalize(const float in[4], float out[4]);

void quaternion_inverse(const float in[4], float out[4]);

void quaternion_conjugate(const float in[4], float out[4]);

float quaternion_abs(const float q[4]);

#endif /* QUATERNION_H */
