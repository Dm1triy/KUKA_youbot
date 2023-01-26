from numba import njit
import numba
import numpy as np
from Pygame_GUI.Space3D.constants import *


@njit(fastmath=True)
def triangle_area(ax, ay, bx, by, cx, cy):
    # return (np.cross(b - a, c - a)) / 2
    abx, aby = bx - ax, by - ay
    acx, acy = cx - ax, cy - ay
    return (abx * acy - aby * acx) / 2


@njit(fastmath=True)
def render_edges(color_mat, depth_mat, vertices, edges, edge_colors):
    for edge in range(len(edges)):
        all_vert = vertices[edges[edge]]
        x0, y0, z0, w0 = all_vert[0]
        x1, y1, z1, w1 = all_vert[1]
        if w0 <= 0 or w1 <= 0:
            continue
        x0, y0, z0 = x0 / w0, y0 / w0,  w0
        x1, y1, z1 = x1 / w1, y1 / w1,  w1

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
                xp = xl * (yp-y0) / yl + x0
                if not 1 > xp > -1:
                    continue
                cx = int((xp + 1) / 2 * WIDTH)
                zp = zl * (yp-y0) / yl + z0
                if depth_mat[cx, cy] > zp or depth_mat[cx, cy] == 0.0:
                    color_mat[cx, cy, :] = edge_colors[edge]
        else:
            for cx in range(bbx_min, bbx_max):
                xp = cx / WIDTH * 2 - 1
                yp = yl * (xp-x0) / xl + y0
                if not 1 > yp > -1:
                    continue
                cy = int((1 - yp) / 2 * HEIGHT)
                zp = zl * (xp-x0) / xl + z0
                if depth_mat[cx, cy] > zp or depth_mat[cx, cy] == 0.0:
                    color_mat[cx, cy] = edge_colors[edge]



@njit(fastmath=True)
def render_polygons(color_mat, depth_mat, vertices, faces, normals, face_colors):
    for face in range(len(faces)):
        all_vert = vertices[faces[face]]
        x0, y0, z0, w0 = all_vert[0]
        x1, y1, z1, w1 = all_vert[1]
        x2, y2, z2, w2 = all_vert[2]
        if w0 <= 0 or w1 <= 0 or w2 <= 0:
            continue
        x0, y0, z0 = x0 / w0, y0 / w0,  w0
        x1, y1, z1 = x1 / w1, y1 / w1,  w1
        x2, y2, z2 = x2 / w2, y2 / w2,  w2
        g_normal = normals[face]
        plane_a = (y0-y1)*(z0-z2)-(z0-z1)*(y0-y2)
        plane_b = -(x0-x1)*(z0-z2)+(z0-z1)*(x0-x2)
        plane_c = (x0-x1)*(y0-y2)-(y0-y1)*(x0-x2)
        plane_d = plane_a * x0 + plane_b * y0 + plane_c * w0
        xp0 = int((x0 + 1) / 2 * WIDTH)
        xp1 = int((x1 + 1) / 2 * WIDTH)
        xp2 = int((x2 + 1) / 2 * WIDTH)
        yp0 = int((1 - y0) / 2 * HEIGHT)
        yp1 = int((1 - y1) / 2 * HEIGHT)
        yp2 = int((1 - y2) / 2 * HEIGHT)
        bbx_min = int(max(0, min(min(xp0, xp1), xp2)))
        bbx_max = int(min(WIDTH - 1, max(max(xp0, xp1), xp2)))
        bby_min = int(max(0, min(min(yp0, yp1), yp2)))
        bby_max = int(min(HEIGHT - 1, max(max(yp0, yp1), yp2)))
        for cx in range(bbx_min, bbx_max):
            for cy in range(bby_min, bby_max):
                if plane_c == 0:
                    continue
                xp = cx / WIDTH * 2 - 1
                yp = 1 - cy / HEIGHT * 2
                dist = -(plane_a * xp + plane_b * yp - plane_d) / plane_c
                if depth_mat[cx, cy] > dist or depth_mat[cx, cy] == 0.0:
                    full_triangle_area = abs(triangle_area(xp0, yp0, xp1, yp1, xp2, yp2))
                    area_sum = 0
                    area_sum += abs(triangle_area(xp0, yp0, xp1, yp1, cx, cy))
                    area_sum += abs(triangle_area(xp0, yp0, xp2, yp2, cx, cy))
                    area_sum += abs(triangle_area(xp1, yp1, xp2, yp2, cx, cy))
                    if area_sum + 0.01 > full_triangle_area > area_sum - 0.01:

                        depth_mat[cx, cy] = dist
                        lighting = (np.dot(g_normal, LIGHT_DIRECTION) /
                                    (np.linalg.norm(LIGHT_DIRECTION, ord=1) * np.linalg.norm(g_normal, ord=1)) + 1) / 2
                        r, g, b = face_colors[face]
                        h, s, l = rgb_to_hsl(r, g, b)
                        color_mat[cx, cy, :] = hsl_to_rgb(h, s, lighting ** 2)


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


@njit(fastmath=True)
def render_func(vertices,
                face_order,
                color_faces,
                faces,
                face_normals,
                face_centers,
                not_drawn_vertices,
                camera_position,
                polygon_arr,
                colors):
    for i in range(len(face_order)):
        index = face_order[i]
        color = color_faces[index]
        face = faces[index]
        normal = face_normals[index]
        corner = face_centers[index][:3]
        # polygon = vertices[face]
        if not (not_drawn_vertices.any() and overlap(face, not_drawn_vertices)):
            if np.dot(normal, (camera_position[:3] - corner)) > 0:
                lighting = (np.dot(normal, LIGHT_DIRECTION) /
                            (np.linalg.norm(LIGHT_DIRECTION, ord=1) * np.linalg.norm(normal, ord=1)) + 1) / 2
                polygon_arr[i, :] = vertices[face]
                colors[i, :] = [*color[:2], int(100 * lighting ** 2), 100]
