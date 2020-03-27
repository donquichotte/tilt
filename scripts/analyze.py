#!/usr/bin/python3

import sys
import matplotlib.pyplot as plt
import numpy as np
import json


def main(fn):
    f = open(fn, "r")
    pitch = []
    roll = []
    quaternion = []
    t = []

    for line in f.readlines():
        l = json.loads(line)
        pitch.append(l["tilt"][0])
        roll.append(l["tilt"][1])
        quaternion.append(l["quaternion"])
        t.append(l["t"])

    plt.figure()
    plt.plot(t, pitch, label="pitch")
    plt.plot(t, roll, label="roll")
    plt.xlabel("t [s]")
    plt.legend()

    plt.figure()
    plt.plot(t, quaternion, label="quaternion")
    plt.xlabel("t [s]")
    plt.legend()
    plt.show()


if __name__ == "__main__":
    main(sys.argv[1])
