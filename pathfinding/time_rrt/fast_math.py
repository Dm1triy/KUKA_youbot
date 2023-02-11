import numpy as np
import scipy
import scipy.spatial
from numba import njit


@njit(fastmath=True)
def fast_closest(target, src, out_ind, out_dist, n, remove_target):
    for p in range(1, len(src)):
        is_target = False
        for rm_t in remove_target:
            if p == rm_t:
                is_target = True
                break
        if is_target:
            continue
        point = src[p]
        dist = np.linalg.norm(point - target)
        for i in range(n):
            if out_dist[i] > dist:
                out_dist[i] = dist
                out_ind[i] = p
                for j in range(n - 1, i, -1):
                    out_ind[j] = out_ind[j - 1]
                    out_dist[j] = out_dist[j - 1]
                break


def find_closest(target, src, n=1, /, dist_limit=None, remove_target=(-1, -1)):
    remove_target = np.array(remove_target).astype(np.int32)
    target = target.astype(np.float32)
    out_ind = np.zeros(n).astype(np.int32)
    out_dist = np.zeros(n).astype(np.float32)
    out_dist[0] = np.linalg.norm(src[0] - target)
    fast_closest(target, src, out_ind, out_dist, n, remove_target)
    return [out_dist[:n], out_ind[:n]]