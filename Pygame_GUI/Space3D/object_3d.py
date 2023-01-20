import numpy as np
import pygame as pg
from Pygame_GUI.Space3D.matrix_functions import *
from numba import njit


@njit(fastmath=True)
def any_func(arr, a, b):
    return np.any((arr == a) | (arr == b))


class Object3D:
    def __init__(self, render, vertices='', faces=''):
        self.render = render
        if vertices:
            self.vertices = np.array([np.array(v) for v in vertices])
        if faces:
            self.faces = np.array([np.array(face) for face in faces])

        self.translate([0.0001, 0.0001, 0.0001])

        self.font = pg.font.SysFont('Arial', 30, bold=True)
        self.color_faces = [(pg.Color('orange'), face) for face in self.faces]
        self.movement_flag, self.draw_vertices = True, False
        self.label = ''
        self.normals = [self.face_normal(i) for i in self.faces]

    def draw(self):
        self.screen_projection()

    def face_normal(self, face):
        A = self.vertices[face[0]][:3] - self.vertices[face[1]][:3]
        B = self.vertices[face[0]][:3] - self.vertices[face[-1]][:3]
        return np.cross(A, B), self.vertices[face[0]][:3]

    def screen_projection(self):
        vertices = self.vertices @ self.render.camera.camera_matrix()
        vertices = vertices @ self.render.projection.projection_matrix
        vertices /= vertices[:, -1].reshape(-1, 1)
        vertices[(vertices > 2) | (vertices < -2)] = 0
        vertices = vertices @ self.render.projection.to_screen_matrix
        vertices = vertices[:, :2]

        for index, color_face in enumerate(self.color_faces):
            color, face = color_face
            polygon = vertices[face]
            if not any_func(polygon, self.render.h_width, self.render.h_height):
                if len(polygon) < 3:
                    pg.draw.aalines(self.render.operating_surf, color, False, polygon)
                else:
                    cam_vect = self.render.camera.transition[:3] - self.normals[index][1]
                    if np.dot(self.normals[index][0], cam_vect) > 0:
                        pg.draw.polygon(self.render.operating_surf, color, polygon, 1)
                if self.label:
                    text = self.font.render(self.label[index], True, pg.Color('white'))
                    self.render.operating_surf.blit(text, polygon[-1])

        if self.draw_vertices:
            for vertex in vertices:
                if not any_func(vertex, self.render.h_width, self.render.h_height):
                    pg.draw.circle(self.render.operating_surf, pg.Color('white'), vertex, 2)

    def translate(self, pos):
        self.vertices = self.vertices @ translate(pos)

    def scale(self, scale_to):
        self.vertices = self.vertices @ scale(scale_to)

    def rotate_x(self, angle):
        self.vertices = self.vertices @ rotate_x(angle)

    def rotate_y(self, angle):
        self.vertices = self.vertices @ rotate_y(angle)

    def rotate_z(self, angle):
        self.vertices = self.vertices @ rotate_z(angle)


class Axes(Object3D):
    def __init__(self, render):

        self.vertices = np.array([(0, 0, 0, 1), (1, 0, 0, 1), (0, 1, 0, 1), (0, 0, 1, 1)])
        self.faces = np.array([(0, 1), (0, 2), (0, 3)])
        self.colors = [pg.Color('red'), pg.Color('green'), pg.Color('blue')]
        self.color_faces = [(color, face) for color, face in zip(self.colors, self.faces)]
        self.draw_vertices = False
        self.label = 'XYZ'
        super().__init__(render)