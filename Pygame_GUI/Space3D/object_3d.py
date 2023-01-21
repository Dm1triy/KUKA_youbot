import numpy as np
import pygame as pg
from Pygame_GUI.Space3D.matrix_functions import *
from numba import njit


@njit(fastmath=True)
def overlap(a, b):
    for i in a:
        for j in b:
            if i == j:
                return True
    return False


LIGHT_DIRECTION = [1, 1, 0]


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
            self.faces = np.array([np.array(face) for face in faces])
        # self.translate([0.0001, 0.0001, 0.0001])
        self.font = pg.font.SysFont('Arial', 30, bold=True)
        self.color_faces = [pg.Color('orange') for _ in self.faces]
        self.movement_flag, self.draw_vertices = True, False
        self.label = ''
        self.normals = [self.face_normal(i) for i in self.faces]
        self.vert_to_global()
        self.not_drawn_vertices = []
        self.min_dist = 3

    def draw(self):
        self.vert_to_global()
        self.screen_projection()

    def face_normal(self, face):
        A = self.vertices[face[0]][:3] - self.vertices[face[1]][:3]
        B = self.vertices[face[0]][:3] - self.vertices[face[-1]][:3]
        return np.cross(B, A), self.vertices[face[0]][:3]

    def vert_to_global(self):
        self.global_vert = self.vertices @ self.transform

    def screen_projection(self):
        self.not_drawn_vertices = []
        vertices = self.global_vert @ self.render.camera.camera_matrix()
        vertices = vertices @ self.render.projection.projection_matrix
        for v in range(len(vertices)):
            if vertices[v][-1] < 0:
                self.not_drawn_vertices.append(v)
                continue
            else:
                vertices[v] /= vertices[v][-1]
                x, y, z = vertices[v][:3]
                md = self.min_dist
                if not(md > x > -md and md > y > -md and md > z > -md):
                    self.not_drawn_vertices.append(v)
        vertices = vertices @ self.render.projection.to_screen_matrix
        vertices = vertices[:, :2]


        for index in range(len(self.faces)):
            color = self.color_faces[index]
            face = self.faces[index]
            normal, corner = self.normals[index]
            polygon = vertices[face]
            if not(self.not_drawn_vertices and overlap(face, np.array(self.not_drawn_vertices))):
                if len(polygon) < 3:
                    pg.draw.aalines(self.render.operating_surf, color, False, polygon)
                else:
                    cam_vect = self.render.camera.position()[:3] - corner
                    if np.dot(normal, cam_vect) > 0:
                        lighting = (np.dot(normal, LIGHT_DIRECTION) /
                                    (np.linalg.norm(LIGHT_DIRECTION, ord=1) * np.linalg.norm(face, ord=1)) + 1) / 2
                        color = list(map(lambda x: max(min(254, (x * lighting).astype(int)), 0), color))
                        pg.draw.polygon(self.render.operating_surf, color, polygon)
                if self.label:
                    text = self.font.render(self.label[index], True, pg.Color('white'))
                    self.render.operating_surf.blit(text, polygon[-1])
        if self.draw_vertices:
            for index in range(len(vertices)):
                vertex = vertices[index]
                if not(self.not_drawn_vertices and overlap(np.array([index]), np.array(self.not_drawn_vertices))):
                    pg.draw.circle(self.render.operating_surf, pg.Color('white'), vertex, 2)


class Axes(Solid3D):
    def __init__(self, render):
        self.vertices = np.array([(0, 0, 0, 1), (1, 0, 0, 1), (0, 1, 0, 1), (0, 0, 1, 1)])
        self.faces = np.array([(0, 1), (0, 2), (0, 3)])
        self.colors = [pg.Color('red'), pg.Color('green'), pg.Color('blue')]
        self.color_faces = [(color, face) for color, face in zip(self.colors, self.faces)]
        self.draw_vertices = False
        self.label = 'XYZ'
        super().__init__(render)
