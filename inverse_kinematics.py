import math
import numpy as np

import numpy as np
from math import *
import time

# 3735.84, 234.4, 140.8

def ik_solver(dx, dy):
    alpha = degrees(atan(dx / (160 - dy)))
    s = sqrt(dx ** 2 + (160 - dy) ** 2)

    b = degrees(acos((s ** 2 - 3735.84) / (200 * s)))
    a = degrees(acos((3735.84 + s ** 2) / (234.4 * s)))
    c = 180 - b - a

    thigh_angle = 90 - (b - alpha)
    tibia_angle = 90 - (alpha + a)

    beta = 140.8 - tibia_angle
    L = sqrt(1376.89 - 1358.4 * cos(radians(beta)))
    Gamma = degrees(acos((224.89 + L ** 2) / (56.6 * L)))
    delta = degrees(acos((L ** 2 - 385) / (48 * L)))

    servo_angle = -135 + Gamma + delta

    return thigh_angle, servo_angle