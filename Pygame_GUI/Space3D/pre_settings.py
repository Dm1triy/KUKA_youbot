import numpy as np
from numba import njit

WIDTH = 300
HEIGHT = 300

jitter = 0.001


NEAR_PLANE = 0.1
FAR_PLANE = 100
float_bit = np.float64
LIGHT_DIRECTION = np.array([1, 0.6, 0.3]).astype(float_bit)

