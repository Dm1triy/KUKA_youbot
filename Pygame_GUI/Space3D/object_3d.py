import numpy as np
import pygame as pg
from Pygame_GUI.Space3D.matrix_functions import *
from Pygame_GUI.Space3D.fast_math import *
import time


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
        ]).astype(float_bit)

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
            self.vertices = np.array([np.array(v) for v in vertices]).astype(float_bit)
        if faces:
            faces_ = []
            for face in faces:
                for f_ in range(2, len(face)):
                    faces_.append([face[f_ - 1], face[f_], face[0]])
            self.faces = np.array(faces_).astype(np.uint32)
        self.center_of_mass = np.mean(self.vertices, axis=0).astype(float_bit)
        self.font = pg.font.SysFont('Arial', 30, bold=True)
        self.color_faces = np.array([[255,255,255] for _ in self.faces]).astype(np.uint32)
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
        self.face_normals = np.array(face_normals).astype(float_bit)
        self.face_centers = np.array(face_centers).astype(float_bit)

        self.vert_to_global()
        self.not_drawn_vertices = []
        self.min_dist = 3
        self.global_face_centers = np.copy(self.face_centers)
        self.global_vert = np.copy(self.vertices)

    def draw(self):
        print("timer", end=" ")
        t = time.time()
        self.vert_to_global()
        vertices = self.global_vert @ self.render.camera.camera_matrix()
        vertices = vertices @ self.render.projection.projection_matrix
        print("t1:", t-time.time(), end=' ')
        t = time.time()
        render_polygons(self.render.color_mat,
                        self.render.depth_mat,
                        vertices,
                        self.faces,
                        self.face_normals,
                        self.color_faces
                        )
        print("end:", t - time.time())
        t = time.time()

    def draw_old(self):
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
        print("1:", time.time() - t, end=" ")
        t = time.time()
        render_func(vertices,
                    face_order[::-1],
                    self.color_faces,
                    self.faces,
                    self.face_normals,
                    self.global_face_centers,
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
