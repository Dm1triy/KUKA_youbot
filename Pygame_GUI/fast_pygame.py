from numba import njit

@njit(fastmath=True)
def fast_blit(disp, surf, x, y):
    xl, yl = surf.shape[:2]
    xm, ym = surf.shape[:2]
    for i in range(x, xl):
        if 0 < i < xm:
            for j in range(y, yl):
                if 0 < i < xm:
                    disp[i, j, :] = surf[i, j, :]
