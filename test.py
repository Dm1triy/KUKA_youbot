from numba import njit
import os
import inspect

@njit(fastmath=True)
def range_cut(mi, ma, num):
    return min(max(num, mi), ma)

print(range_cut)