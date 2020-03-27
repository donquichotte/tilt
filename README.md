# Tilt Estimator Using 6DOF IMU

## Introduction

Implementation of a complementary filter estimating tilt, i.e. pitch and roll, from accelerometers and gyroscopes, as described in [1].

The algorithm has been extended with arbitrary nominal gravity vectors and gravity vector calibration.

Suitable for systems that are not subjects to other accelerations than gravity, e.g. head trackers.

## Build

```
cd src
make
```

## Run

```
./tilt
```


[1] [https://stanford.edu/class/ee267/notes/ee267_notes_imu.pdf](https://stanford.edu/class/ee267/notes/ee267_notes_imu.pdf)