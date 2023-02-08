import numpy as np
from numba import njit

from Pygame_GUI.Space3D.pre_settings import *


@njit(fastmath=True)
def triangle_area(ax, ay, bx, by, cx, cy):
    # return (np.cross(b - a, c - a)) / 2
    abx, aby = bx - ax, by - ay
    acx, acy = cx - ax, cy - ay
    return (abx * acy - aby * acx) / 2


@njit(fastmath=True)
def range_cut(mi, ma, num):
    return min(max(num, mi), ma)


@njit(fastmath=True)
def set_pixel(color_mat, depth_mat, priority_mat, x, y, color, depth, priority):
    color_mat[x, y, :] = color
    depth_mat[x, y] = depth
    priority_mat[x, y] = priority


@njit(fastmath=True)
def render_vertices(color_mat, depth_mat, priority_mat, vertices, vertex_colors, vertex_radius):
    for vertex in range(len(vertices)):
        x0, y0, z0, w0 = vertices[vertex]
        if w0 <= 0:
            continue
        x0, y0, z0 = x0 / w0, y0 / w0, w0
        xp0 = int((x0 + 1) / 2 * WIDTH)
        yp0 = int((1 - y0) / 2 * HEIGHT)
        if not (0 <= xp0 < WIDTH and 0 <= yp0 < HEIGHT):
            continue
        rad = vertex_radius[vertex]
        dist = z0
        if depth_mat[xp0, yp0] + jitter > dist > depth_mat[xp0, yp0] - jitter:
            if priority_mat[xp0, yp0] < dist and priority_mat[xp0, yp0] != 0:
                continue
        elif depth_mat[xp0, yp0] < dist and depth_mat[xp0, yp0] != 0:
            continue
        for dx in range(-rad, rad):
            for dy in range(-rad, rad):
                cx, cy = xp0 + dx, yp0 + dy
                if 0 <= cx < WIDTH and 0 <= cy < HEIGHT and dx ** 2 + dy ** 2 < rad ** 2:
                    set_pixel(color_mat, depth_mat, priority_mat, cx, cy, vertex_colors[vertex], z0, z0)
                    # color_mat[cx, cy, :] = vertex_colors[vertex]
                    # depth_mat[cx, cy] = z0
                    # priority_mat[cx, cy] = z0


@njit(fastmath=True)
def render_edges(color_mat, depth_mat, priority_mat, vertices, edges, edge_colors, thickness):
    for edge in range(len(edges)):
        all_vert = vertices[edges[edge]]
        th = thickness[edge]
        x0, y0, z0, w0 = all_vert[0]
        x1, y1, z1, w1 = all_vert[1]
        if w0 <= 0 or w1 <= 0:
            continue
        x0, y0, z0 = x0 / w0, y0 / w0, w0
        x1, y1, z1 = x1 / w1, y1 / w1, w1

        xl, yl, zl = x1 - x0, y1 - y0, z1 - z0
        xp0 = int((x0 + 1) / 2 * WIDTH)
        xp1 = int((x1 + 1) / 2 * WIDTH)
        yp0 = int((1 - y0) / 2 * HEIGHT)
        yp1 = int((1 - y1) / 2 * HEIGHT)
        bbx_min = int(max(0, min(xp0, xp1)))
        bbx_max = int(min(WIDTH - 1, max(xp0, xp1)))
        bby_min = int(max(0, min(yp0, yp1)))
        bby_max = int(min(HEIGHT - 1, max(yp0, yp1)))
        if bby_max - bby_min > bbx_max - bbx_min:
            for cy in range(bby_min, bby_max):
                yp = 1 - cy / HEIGHT * 2
                xp = xl * (yp - y0) / yl + x0
                if not 1 > xp > -1:
                    continue
                cx = int((xp + 1) / 2 * WIDTH)
                dist = zl * (yp - y0) / yl + z0
                if depth_mat[cx, cy] + jitter > dist > depth_mat[cx, cy] - jitter:
                    if priority_mat[cx, cy] < dist and priority_mat[cx, cy] != 0:
                        continue
                elif depth_mat[cx, cy] < dist and depth_mat[cx, cy] != 0:
                    continue
                for t in range(-th // 2, th // 2):
                    ind = range_cut(0, WIDTH - 1, int(cx + t))
                    set_pixel(color_mat, depth_mat, priority_mat, ind, cy, edge_colors[edge], dist, dist)
                    # color_mat[ind, cy, :] = edge_colors[edge]
                    # color_mat[ind, cy, :] = [zp * 50, zp * 50, zp * 50]
                    # depth_mat[ind, cy] = zp
                    # priority_mat[cx, cy] = zp
        else:
            for cx in range(bbx_min, bbx_max):
                xp = cx / WIDTH * 2 - 1
                yp = yl * (xp - x0) / xl + y0
                if not 1 > yp > -1:
                    continue
                cy = int((1 - yp) / 2 * HEIGHT)
                dist = zl * (xp - x0) / xl + z0
                if depth_mat[cx, cy] + jitter > dist > depth_mat[cx, cy] - jitter:
                    if priority_mat[cx, cy] < dist and priority_mat[cx, cy] != 0:
                        continue
                elif depth_mat[cx, cy] < dist and depth_mat[cx, cy] != 0:
                    continue
                for t in range(-th // 2, th // 2):
                    ind = range_cut(0, HEIGHT - 1, int(cy + t))
                    set_pixel(color_mat, depth_mat, priority_mat, cx, ind, edge_colors[edge], dist, dist)
                    # color_mat[cx, ind, :] = edge_colors[edge]
                    # color_mat[cx, ind, :] = [zp*50, zp*50, zp*50]

                    # depth_mat[cx, ind] = zp
                    # priority_mat[cx, cy] = z0


@njit(fastmath=True)
def render_polygons(color_mat, depth_mat, priority_mat, vertices, faces, normals, face_colors):
    for face in range(len(faces)):
        all_vert = vertices[faces[face]]
        x0, y0, z0, w0 = all_vert[0]
        x1, y1, z1, w1 = all_vert[1]
        x2, y2, z2, w2 = all_vert[2]
        if (x0 - x1) * (y0 - y2) - (y0 - y1) * (x0 - x2) > 0:
            continue
        neg_points = np.array([[0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]]).astype(float_bit)
        positive_points = np.array([[0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]]).astype(float_bit)
        negative_counter = 0
        positive_counter = 0
        vert = np.zeros((3, 3)).astype(float_bit)
        if w0 <= NEAR_PLANE:
            neg_points[negative_counter] = all_vert[0]
            negative_counter += 1
        else:
            positive_points[positive_counter] = all_vert[0]
            positive_counter += 1

        if w1 <= NEAR_PLANE:
            neg_points[negative_counter] = all_vert[1]
            negative_counter += 1
        else:
            positive_points[positive_counter] = all_vert[1]
            positive_counter += 1

        if w2 <= NEAR_PLANE:
            neg_points[negative_counter] = all_vert[2]
            negative_counter += 1
        else:
            positive_points[positive_counter] = all_vert[2]
            positive_counter += 1

        if negative_counter == 3:
            continue
        elif negative_counter == 2:
            x0, y0, z0, w0 = positive_points[0]
            x1, y1, z1, w1 = neg_points[0]
            x2, y2, z2, w2 = neg_points[1]
            z0, z1, z2 = w0, w1, w2

            x1, y1 = (NEAR_PLANE - z0) / (z1 - z0) * (x1 - x0) + x0, (NEAR_PLANE - z0) / (z1 - z0) * (y1 - y0) + y0
            x2, y2 = (NEAR_PLANE - z0) / (z2 - z0) * (x2 - x0) + x0, (NEAR_PLANE - z0) / (z2 - z0) * (y2 - y0) + y0
            z1 = z2 = NEAR_PLANE
            # x0, y0 = x0 / z0, y0 / z0
            # x1, y1 = x1 / z1, y1 / z1
            # x2, y2 = x2 / z2, y2 / z2
            vert[0, 0], vert[0, 1], vert[0, 2], vert[1, 0], vert[1, 1], vert[1, 2], vert[2, 0], vert[2, 1], vert[
                2, 2] = x0, y0, z0, x1, y1, z1, x2, y2, z2
            draw_face(color_mat, depth_mat, priority_mat, vert, normals[face], face_colors[face])

        elif negative_counter == 1:
            x0, y0, z0, w0 = neg_points[0]
            x1, y1, z1, w1 = positive_points[0]
            x2, y2, z2, w2 = positive_points[1]
            z0, z1, z2 = w0, w1, w2

            x3, y3 = (NEAR_PLANE - z0) / (z1 - z0) * (x1 - x0) + x0, (NEAR_PLANE - z0) / (z1 - z0) * (y1 - y0) + y0
            x4, y4 = (NEAR_PLANE - z0) / (z2 - z0) * (x2 - x0) + x0, (NEAR_PLANE - z0) / (z2 - z0) * (y2 - y0) + y0
            z3 = z4 = NEAR_PLANE
            # x1, y1 = x1 / z1, y1 / z1
            # x2, y2 = x2 / z2, y2 / z2
            # x3, y3 = x3 / z3, y3 / z3
            # x4, y4 = x4 / z4, y4 / z4

            vert[0, 0], vert[0, 1], vert[0, 2], vert[1, 0], vert[1, 1], vert[1, 2], vert[2, 0], vert[2, 1], vert[
                2, 2] = x1, y1, z1, x2, y2, z2, x3, y3, z3
            draw_face(color_mat, depth_mat, priority_mat, vert, normals[face], face_colors[face])
            vert[0, 0], vert[0, 1], vert[0, 2], vert[1, 0], vert[1, 1], vert[1, 2], vert[2, 0], vert[2, 1], vert[
                2, 2] = x2, y2, z2, x3, y3, z3, x4, y4, z4
            draw_face(color_mat, depth_mat, priority_mat, vert, normals[face], face_colors[face])
        elif negative_counter == 0:
            x0, y0, z0, w0 = positive_points[0]
            x1, y1, z1, w1 = positive_points[1]
            x2, y2, z2, w2 = positive_points[2]
            z0, z1, z2 = w0, w1, w2
            # x0, y0 = x0 / z0, y0 / z0
            # x1, y1 = x1 / z1, y1 / z1
            # x2, y2 = x2 / z2, y2 / z2

            vert[0, 0], vert[0, 1], vert[0, 2], vert[1, 0], vert[1, 1], vert[1, 2], vert[2, 0], vert[2, 1], vert[
                2, 2] = x0, y0, z0, x1, y1, z1, x2, y2, z2
            draw_face(color_mat, depth_mat, priority_mat, vert, normals[face], face_colors[face])


@njit(fastmath=True)
def draw_face(color_mat, depth_mat, priority_mat, vertices, g_normal, face_color):
    x0, y0, z0 = vertices[0]
    x1, y1, z1 = vertices[1]
    x2, y2, z2 = vertices[2]
    cent = (z0 + z1 + z2) / 3
    x0, y0 = x0 / z0, y0 / z0
    x1, y1 = x1 / z1, y1 / z1
    x2, y2 = x2 / z2, y2 / z2
    plane_a = (y0 - y1) * (z0 - z2) - (z0 - z1) * (y0 - y2)
    plane_b = -(x0 - x1) * (z0 - z2) + (z0 - z1) * (x0 - x2)
    plane_c = (x0 - x1) * (y0 - y2) - (y0 - y1) * (x0 - x2)
    plane_d = plane_a * x0 + plane_b * y0 + plane_c * z0
    xp0 = int((x0 + 1) / 2 * WIDTH)
    xp1 = int((x1 + 1) / 2 * WIDTH)
    xp2 = int((x2 + 1) / 2 * WIDTH)
    yp0 = int((1 - y0) / 2 * HEIGHT)
    yp1 = int((1 - y1) / 2 * HEIGHT)
    yp2 = int((1 - y2) / 2 * HEIGHT)

    if plane_c == 0:
        return
    bbx_min = int(max(0, min(min(xp0, xp1), xp2)))
    bbx_max = int(min(WIDTH - 1, max(max(xp0, xp1), xp2)))
    bby_min = int(max(0, min(min(yp0, yp1), yp2)))
    bby_max = int(min(HEIGHT - 1, max(max(yp0, yp1), yp2)))
    lighting = (np.dot(g_normal, LIGHT_DIRECTION) / (
            np.linalg.norm(LIGHT_DIRECTION, ord=1) * np.linalg.norm(g_normal, ord=1)) + 1) / 2
    r, g, b = face_color
    h, s, l = rgb_to_hsl(r, g, b)
    r, g, b = hsl_to_rgb(h, s, lighting ** 2)

    for cx in range(bbx_min, bbx_max):
        for cy in range(bby_min, bby_max):
            xp = float(cx) * 2 / WIDTH - 1
            yp = 1 - float(cy) * 2 / HEIGHT
            dist = -(plane_a * xp + plane_b * yp - plane_d) / plane_c
            if depth_mat[cx, cy] + jitter > dist > depth_mat[cx, cy] - jitter:
                if priority_mat[cx, cy] < cent and priority_mat[cx, cy] != 0:
                    continue
            elif depth_mat[cx, cy] < dist and depth_mat[cx, cy] != 0:
                continue
            full_triangle_area = abs(triangle_area(xp0, yp0, xp1, yp1, xp2, yp2))
            area_sum = 0
            area_sum += abs(triangle_area(xp0, yp0, xp1, yp1, cx, cy))
            area_sum += abs(triangle_area(xp0, yp0, xp2, yp2, cx, cy))
            area_sum += abs(triangle_area(xp1, yp1, xp2, yp2, cx, cy))
            color = np.array([r, g, b])
            if area_sum + 0.01 > full_triangle_area > area_sum - 0.01:
                set_pixel(color_mat, depth_mat, priority_mat, cx, cy, color, dist, cent)
                # depth_mat[cx, cy] = dist
                # color_mat[cx, cy, :] = [r, g, b]
                # color_mat[cx, cy, :] = [int(20*dist), int(20*dist), int(20*dist)]


@njit(fastmath=True)
def rgb_to_hsl(r, g, b):
    high = max(r, g, b)
    low = min(r, g, b)
    l = ((high + low) / 2)

    if high == low:
        h = 0.0
        s = 0.0
    else:
        d = high - low
        s = d / (2 - high - low) if l > 0.5 else d / (high + low)
        h = {
            r: (g - b) / d + (6 if g < b else 0),
            g: (b - r) / d + 2,
            b: (r - g) / d + 4,
        }[high]
        h /= 6
    l /= 255

    return h, s, l


@njit(fastmath=True)
def hsl_to_rgb(h, s, l):
    def hue_to_rgb(p, q, t):
        t += 1 if t < 0 else 0
        t -= 1 if t > 1 else 0
        if t < 1 / 6: return p + (q - p) * 6 * t
        if t < 1 / 2: return q
        if t < 2 / 3: p + (q - p) * (2 / 3 - t) * 6
        return p

    if s == 0:
        r, g, b = l, l, l
    else:
        q = l * (1 + s) if l < 0.5 else l + s - l * s
        p = 2 * l - q
        r = hue_to_rgb(p, q, h + 1 / 3)
        g = hue_to_rgb(p, q, h)
        b = hue_to_rgb(p, q, h - 1 / 3)

    return int(r * 255), int(g * 255), int(b * 255)


@njit(fastmath=True)
def overlap(a, b):
    for i in a:
        for j in b:
            if i == j:
                return True
    return False


@njit(fastmath=True)
def face_fast_sort(ref, points):
    return np.argsort(np.array(list([np.linalg.norm(ref - points[i][:3]) for i in range(len(points))])))


@njit(fastmath=True)
def detect_not_drawn_vertices(vertices, md, not_drawn_vertices):
    l = 0
    for v in range(len(vertices)):
        if vertices[v][-1] < 0:
            not_drawn_vertices[l] = v
            l += 1
            continue
        else:
            vertices[v] /= vertices[v][-1]
            x, y, z = vertices[v][:3]
            if not (md > x > -md and md > y > -md and md > z > -md):
                not_drawn_vertices[l] = v
                l += 1
    return l
