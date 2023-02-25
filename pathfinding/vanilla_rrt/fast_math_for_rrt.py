import numpy as np
import scipy
import scipy.spatial
from numba import njit

@njit(fastmath=True)
def check_obstacle(point1, point2, bool_map, growth_factor, e, info):
    shift_vector = (point2.astype(np.float64) - point1.astype(np.float64)).astype(np.float64)
    info[:3], info[3], info[4] = np.zeros_like(point1), -1, 0
    transition = np.linalg.norm(shift_vector[:2])
    iters = int(transition * 2)
    if iters == 0:
        return
    shift_vector = shift_vector / iters
    all_shift = shift_vector
    point_set = False
    c_point = np.zeros_like(point1).astype(np.int32)
    c_p = np.zeros_like(point1).astype(np.float64)
    iters_made = 0
    for i in range(1, iters + 1):
        iters_made = i
        all_shift = np.copy(shift_vector * i)
        c_p = point1 + shift_vector * i
        point_set = True
        for cp in range(len(c_p)):
            c_point[cp] = int(c_p[cp])
        if bool_map[c_point[0], c_point[1], c_point[2]]:
            i -= 1
            iters_made = i
            break
        if abs(np.linalg.norm(shift_vector * i)) >= growth_factor:
            break
    if np.linalg.norm(all_shift) < e or not point_set:
        info[:3], info[3], info[4] = np.zeros_like(point1), -1, 0
    elif iters_made > 1:
        info[:3], info[3], info[4] = c_p, np.linalg.norm(all_shift), iters_made == iters
    else:
        info[:3], info[3], info[4] = np.zeros_like(point1), -1, 0

@njit(fastmath=True)
def fast_closest(target, src, out_ind, out_dist, n, inv):
    for p in range(1, len(src)):
        point = src[p]
        delta = (target - point)*inv
        if delta[2] == 0:
            continue
        dist_total = np.linalg.norm(delta)
        for i in range(n):
            if out_dist[i] > dist_total:
                for j in range(n - 1, i, -1):
                    out_ind[j] = out_ind[j - 1]
                    out_dist[j] = out_dist[j - 1]
                out_dist[i] = dist_total
                out_ind[i] = p
                break


