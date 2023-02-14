import numpy as np
import pygame as pg
from Pygame_GUI.Space3D.matrix_functions import *
from Pygame_GUI.Space3D.fast_math import *
import time


class Object3D:
    def __init__(self, render, /,
                 vertices=None, vertex_radius=None, vertex_colors=None,
                 edges=None, edges_thickness=None, color_edges=None,
                 faces=None, color_faces=None,
                 pos=None):

        if not pos:
            pos = [0, 0, 0]
        self.render = render

        x, y, z = pos
        self.transform = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [x, y, z, 1]
        ]).astype(float_bit)

        self.global_vert = np.array(False)
        self.global_center_of_mass = np.array(False)

        self.draw_vertices = True
        self.draw_edges = True
        self.draw_faces = True

        self.empty = True

        if not isinstance(vertices, np.ndarray):
            vertices = np.array([np.array(v) for v in vertices]).astype(float_bit)
        if vertices.any():
            self.vertices = np.array([np.array(v).astype(float_bit) for v in vertices]).astype(float_bit)
            if vertex_colors:
                self.vertex_colors = np.array(vertex_colors).astype(np.int16)
            else:
                self.vertex_colors = np.array([[255, 255, 255] for _ in self.vertices]).astype(np.int32)
            if vertex_radius:
                self.vertex_radius = np.array(vertex_radius).astype(np.int16)
            else:
                self.vertex_radius = np.array([10] * len(self.vertices)).astype(np.int16)

            self.global_vert = np.copy(self.vertices)
            self.center_of_mass = np.mean(self.vertices, axis=0).astype(float_bit)
            self.empty = False
        else:
            self.draw_vertices = False
            self.draw_edges = False
            self.draw_faces = False

        if edges:
            self.edges = np.array(edges).astype(np.uint32)
            if edges_thickness:
                self.edges_thickness = np.array(edges_thickness).astype(np.int16)
            else:
                self.edges_thickness = np.array([1] * len(self.edges)).astype(np.int16)

            if color_edges:
                self.color_edges = np.array(color_edges).astype(np.int32)
            else:
                self.color_edges = np.array([[255, 255, 255] for _ in self.edges]).astype(np.int32)
        else:
            self.draw_edges = False

        if faces:
            faces_ = []
            for face in faces:
                for f_ in range(2, len(face)):
                    faces_.append([face[f_ - 1], face[f_], face[0]])
            self.faces = np.array(faces_).astype(np.uint32)
            if color_faces:
                self.color_faces = np.array(color_faces).astype(np.uint32)
            else:
                self.color_faces = np.array([[255, 255, 255] for _ in self.faces]).astype(np.uint32)
            face_normals = []
            for face in self.faces:
                A = self.vertices[face[0]][:3] - self.vertices[face[-1]][:3]
                B = self.vertices[face[0]][:3] - self.vertices[face[1]][:3]
                face_normals.append(np.cross(B, A))
            self.face_normals = np.array(face_normals).astype(float_bit)
        else:
            self.draw_faces = False



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

    def vert_to_global(self):
        if not self.empty:
            self.global_vert = self.vertices @ self.transform
            self.global_center_of_mass = self.center_of_mass @ self.transform

    def draw(self, workspace):
        if self.empty:
            return
        vertices = self.global_vert @ self.render.camera.camera_matrix()
        vertices = vertices @ self.render.projection.projection_matrix
        if self.draw_vertices:
            render_vertices(*workspace,
                            vertices,
                            self.vertex_colors,
                            self.vertex_radius)
        if self.draw_edges:


            render_edges(*workspace,
                         vertices,
                         self.edges,
                         self.color_edges,
                         self.edges_thickness
                         )
        if self.draw_faces:
            render_polygons(*workspace,
                            vertices,
                            self.faces,
                            self.face_normals,
                            self.color_faces
                            )


class Hollow3D(Object3D):
    def __init__(self, render, vertices='', edges='',  **kwargs):
        super().__init__(render, vertices=vertices, edges=edges, **kwargs)


class Solid3D(Object3D):
    def __init__(self, render, vertices='', faces='',  **kwargs):
        super().__init__(render, vertices=vertices, faces=faces, **kwargs)
        self.draw_vertices = False
        self.draw_edges = False


class Axes(Hollow3D):
    def __init__(self, render):
        super().__init__(render, [(0, 0, 0, 1), (1, 0, 0, 1), (0, 1, 0, 1), (0, 0, 1, 1)], [(0, 1), (0, 2), (0, 3)])
        self.color_edges = np.array([[255, 0, 0], [0, 255, 0], [0, 0, 255]]).astype(np.uint32)
        self.draw_vertices = False
        self.label = 'XYZ'
