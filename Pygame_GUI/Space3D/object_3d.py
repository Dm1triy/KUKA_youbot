import numpy as np
import pygame as pg
from Pygame_GUI.Space3D.matrix_functions import *
from numba import njit
import time


@njit(fastmath=True)
def overlap(a, b):
    for i in a:
        for j in b:
            if i == j:
                return True
    return False


dtype = [('dist', float), ('ind', int)]


@njit(fastmath=True)
def face_fast_sort(ref, points):
    return np.argsort(np.array(list([np.linalg.norm(ref - points[i][:3]) for i in range(len(points))])))


@njit(fastmath=True)
def detect_not_drawn_vertices(vertices, md, not_drawn_vertices):
    l = 0
    for v in range(len(vertices)):
        if vertices[v][-1] < 0:
            not_drawn_vertices[l] = v
            l+=1
            continue
        else:
            vertices[v] /= vertices[v][-1]
            x, y, z = vertices[v][:3]
            if not (md > x > -md and md > y > -md and md > z > -md):
                not_drawn_vertices[l] = v
                l+=1
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


LIGHT_DIRECTION = np.array([1, 1, 0]).astype(np.float64)


class Object3D:
    def __init__(self, render, pos=None):
        if not pos:
            pos = [0, 0, 0]
        self.render = render
        self.font = pg.font.SysFont('Arial', 30, bold=True)
        self.label = ''
        x, y, z = pos
        self.transform = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [x, y, z, 1]
        ])

    def translate(self, pos):
        self.transform = self.transform @ translate(pos)

    def scale(self, scale_to):
        self.transform = self.transform @ scale(scale_to)

    def rotate_x(self, angle):
        self.transform = self.transform @ rotate_x(angle)

    def rotate_y(self, angle):
        self.transform = self.transform @ rotate_y(angle)

    def rotate_z(self, angle):
        self.transform = self.transform @ rotate_z(angle)

    def position(self):
        return self.transform[:3, 3]


class Solid3D(Object3D):
    def __init__(self, render, vertices='', faces='', pos=None):
        super().__init__(render, pos)
        if vertices:
            self.vertices = np.array([np.array(v) for v in vertices])
        if faces:
            faces_ = []
            for face in faces:
                for f_ in range(2, len(face)):
                    faces_.append([face[f_ - 1], face[f_], face[0]])
            self.faces = np.array(faces_)
        self.center_of_mass = np.mean(self.vertices, axis=0)
        self.font = pg.font.SysFont('Arial', 30, bold=True)
        self.color_faces = np.array([pg.Color('white').hsla for _ in self.faces])
        self.movement_flag, self.draw_vertices = True, False
        self.label = ''

        # calculate face normals and centers
        face_normals = []
        face_centers = []
        for face in self.faces:
            center_of_face = np.mean(self.vertices[face], axis=0)
            A = self.vertices[face[0]][:3] - self.vertices[face[-1]][:3]
            B = self.vertices[face[0]][:3] - self.vertices[face[1]][:3]
            face_normals.append(np.cross(B, A))
            face_centers.append(center_of_face)
        self.face_normals = np.array(face_normals).astype(np.float64)
        self.face_centers = np.array(face_centers).astype(np.float64)

        self.vert_to_global()
        self.not_drawn_vertices = []
        self.min_dist = 3
        self.global_face_centers = np.copy(self.face_centers)
        self.global_vert = np.copy(self.vertices)

    def draw(self):
        print("timer")
        t = time.time()
        self.vert_to_global()
        vertices = self.global_vert @ self.render.camera.camera_matrix()
        vertices = vertices @ self.render.projection.projection_matrix

        not_drawn_vertices_buff = np.zeros(len(vertices))
        l = detect_not_drawn_vertices(vertices, self.min_dist, not_drawn_vertices_buff)
        not_drawn_vertices = not_drawn_vertices_buff[:l]

        vertices = vertices @ self.render.projection.to_screen_matrix
        vertices = vertices[:, :2]
        face_order = face_fast_sort(self.render.camera.position()[:3], self.global_face_centers)

        polygons = np.zeros((len(self.faces), 3, 2)).astype(int)
        colors = np.zeros((len(self.faces), 4))
        print("1:", time.time()-t, end=" ")
        t = time.time()
        render_func(vertices,
                    face_order[::-1],
                    self.color_faces,
                    self.faces,
                    self.face_normals,
                    self.face_centers,
                    not_drawn_vertices,
                    self.render.camera.position(),
                    polygons,
                    colors)
        print("2:", time.time() - t, end=" ")
        t = time.time()
        for i in range(polygons.shape[0]):
            color = pg.Color('white')
            color.hsla = colors[i]
            pg.draw.polygon(self.render.operating_surf, color, polygons[i])
        print("3:", time.time() - t)

    def vert_to_global(self):
        self.global_vert = self.vertices @ self.transform
        self.global_center_of_mass = self.center_of_mass @ self.transform
        self.global_face_centers = self.face_centers @ self.transform



class Axes(Solid3D):
    def __init__(self, render):
        self.vertices = np.array([(0, 0, 0, 1), (1, 0, 0, 1), (0, 1, 0, 1), (0, 0, 1, 1)])
        self.faces = np.array([(0, 1), (0, 2), (0, 3)])
        self.colors = [pg.Color('red'), pg.Color('green'), pg.Color('blue')]
        self.color_faces = [(color, face) for color, face in zip(self.colors, self.faces)]
        self.draw_vertices = False
        self.label = 'XYZ'
        super().__init__(render)
