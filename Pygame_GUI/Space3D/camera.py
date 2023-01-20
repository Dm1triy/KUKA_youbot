import numpy as np
import pygame as pg
from Pygame_GUI.Space3D.matrix_functions import *


class Camera:
    def __init__(self, render, position):
        self.render = render
        self.position = np.array([*position, 1.0])
        self.transition = np.array([0, 0, 0, 0])
        self.rotation = [0, 0]
        self.forward = np.array([0, 0, 1, 1])
        self.up = np.array([0, 1, 0, 1])
        self.right = np.array([1, 0, 0, 1])
        self.h_fov = math.pi / 3
        self.v_fov = self.h_fov * (render.height / render.width)
        self.near_plane = 0.1
        self.far_plane = 100
        self.moving_speed = 0.3
        self.rotation_speed = 0.011
        self.mode = 1
        self.direction = np.eye(4)

    def control(self, /, transition=None, rotation=None):
        inv = 1
        if self.mode == 1:
            inv = -1
        if isinstance(transition, np.ndarray):
            self.transition = transition*self.moving_speed
        if rotation:
            self.rotation = rotation
        else:
            self.rotation = [0, 0, 0]
        if self.mode == 0:
            self.position += self.transition @ self.direction * self.moving_speed
        elif self.mode ==1:
            self.position += self.transition * self.moving_speed
        if self.rotation[0]:
            self.direction = (rotate_y(self.rotation[0] * self.rotation_speed * inv)) @ self.direction
        if self.rotation[1]:
            self.direction = (rotate_x(self.rotation[1] * self.rotation_speed * inv)) @ self.direction
        if self.rotation[2]:
            self.direction = (rotate_z(self.rotation[2] * self.rotation_speed * inv)) @ self.direction

    def axiiIdentity(self):
        self.forward = np.array([0, 0, 1, 1])
        self.up = np.array([0, 1, 0, 1])
        self.right = np.array([1, 0, 0, 1])

    def camera_update_axii(self):
        # rotate = rotate_y(self.angleYaw) @ rotate_x(self.anglePitch)
        rotate = self.direction
        self.axiiIdentity()
        self.forward = self.forward @ rotate
        self.right = self.right @ rotate
        self.up = self.up @ rotate

    def camera_matrix(self):
        self.camera_update_axii()
        if self.mode == 0:
            return self.translate_matrix() @ self.rotate_matrix()
        elif self.mode == 1:
            return self.rotate_matrix() @ self.translate_matrix()

    def translate_matrix(self):
        x, y, z, w = self.position
        return np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [-x, -y, -z, 1]
        ])

    def rotate_matrix(self):
        rx, ry, rz, w = self.right
        fx, fy, fz, w = self.forward
        ux, uy, uz, w = self.up
        return np.array([
            [rx, ux, fx, 0],
            [ry, uy, fy, 0],
            [rz, uz, fz, 0],
            [0, 0, 0, 1]
        ])