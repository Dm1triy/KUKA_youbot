import numpy as np
import scipy
import scipy.spatial
from numba import njit


@njit(fastmath=True)
def check_obstacle(point1, point2, bool_map, growth_factor, e, min_speed, max_speed, info):
    map_shape = bool_map.shape
    total_shift_vector = (point2.astype(np.float64) - point1.astype(np.float64)).astype(np.float64)
    info[:3], info[3], info[4] = np.zeros_like(point1), -1, 0
    transition = np.linalg.norm(total_shift_vector[:2])
    time = total_shift_vector[-1]

    if time <= 0:
        return
    speed = transition/time

    norm_shift = total_shift_vector/np.amax(np.absolute(total_shift_vector))

    iters = round(np.amax(total_shift_vector))

    if not min_speed <= speed <= max_speed or iters == 0:
        return


    all_shift = norm_shift
    c_point = np.zeros_like(point1).astype(np.float64)
    c_point_int = np.zeros_like(point1).astype(np.int32)
    iters_made = 0
    for i in range(1, iters + 1):
        iters_made = i
        all_shift = np.copy(norm_shift * i)
        c_point = point1 + all_shift
        for cp in range(len(c_point)):
            c_point_int[cp] = int(c_point[cp])
        if bool_map[c_point_int[0], c_point_int[1], c_point_int[2]]:
            i -= 1
            iters_made = i
            all_shift = np.copy(norm_shift * i)
            c_point = point1 + all_shift
            break
        if abs(np.linalg.norm(all_shift)) >= growth_factor:
            break
    if np.linalg.norm(all_shift[:2])/speed > map_shape[2]:
        return
    if iters_made > 0 and np.linalg.norm(all_shift) > e:
        info[0], info[1] = int(c_point[0]), int(c_point[1])
        info[2] = point1[-1] + np.linalg.norm(all_shift[:2])/speed
        info[3], info[4] = np.linalg.norm(all_shift), iters_made == iters
@njit(fastmath=True)
def check_obstacle_old(point1, point2, bool_map, growth_factor, e, min_speed, max_speed, info):
    total_shift_vector = (point2.astype(np.float64) - point1.astype(np.float64)).astype(np.float64)
    info[:3], info[3], info[4] = np.zeros_like(point1), -1, 0
    transition = np.linalg.norm(total_shift_vector[:2])
    time = total_shift_vector[2]
    iters = int(transition * 2)

    if time <= 0:
        return
    if not min_speed <= transition / time <= max_speed:
        return
    if iters == 0:
        return
    shift_vector = total_shift_vector / iters
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
def fast_closest(target, src, out_ind, out_dist, n, remove_target, remove_mod, max_speed, min_speed):
    for p in range(1, len(src)):
        is_target = False
        point = src[p]
        delta = target - point
        for rm_t in remove_target:
            if p == rm_t:

                if remove_mod:
                    is_target = True
                else:
                    delta = point - target
                break
        if is_target:
            continue
        if delta[2] == 0:
            continue
        dist_total = np.linalg.norm(delta)
        dist = np.linalg.norm(delta[:2])
        speed = dist/delta[2]
        for i in range(n):
            if out_dist[i] > dist_total and max_speed > speed > min_speed:
                for j in range(n - 1, i, -1):
                    out_ind[j] = out_ind[j - 1]
                    out_dist[j] = out_dist[j - 1]
                out_dist[i] = dist_total
                out_ind[i] = p
                break


