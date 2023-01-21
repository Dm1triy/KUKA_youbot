import numpy as np
import pygame as pg
from Pygame_GUI.Space3D.matrix_functions import *


class Camera:
    def __init__(self, render, position):
        self.render = render
        self.init_pos = position
        self.pos_serv = np.array([*position, 1.0])
        self.transition = np.array([0, 0, 0, 0])
        self.rotation = [0, 0]
        self.h_fov = math.pi / 3
        self.v_fov = self.h_fov * (render.height / render.width)
        self.near_plane = 0.1
        self.far_plane = 100
        self.moving_speed = 0.3
        self.zoom_speed = 3
        self.rotation_speed = 0.011
        self.mode = 1
        self.direction = np.eye(4)

    def reset(self):
        self.pos_serv = np.array([*self.init_pos, 1.0])
        self.transition = np.array([0, 0, 0, 0])
        self.rotation = [0, 0]
        self.near_plane = 0.1
        self.far_plane = 100
        self.moving_speed = 0.3
        self.zoom_speed = 3
        self.rotation_speed = 0.011
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
            self.pos_serv += self.transition @ self.direction * self.moving_speed
        elif self.mode == 1 and isinstance(transition, np.ndarray):
            self.pos_serv += self.transition * self.zoom_speed

        if self.rotation[0]:
            self.direction = (rotate_y(self.rotation[0] * self.rotation_speed * inv)) @ self.direction
        if self.rotation[1]:
            self.direction = (rotate_x(self.rotation[1] * self.rotation_speed * inv)) @ self.direction
        if self.rotation[2]:
            self.direction = (rotate_z(self.rotation[2] * self.rotation_speed * inv)) @ self.direction

    def camera_matrix(self):
        if self.mode == 0:
            return self.translate_matrix() @ self.direction.T
        elif self.mode == 1:
            return self.direction.T @ self.translate_matrix()

    def translate_matrix(self):
        x, y, z, w = self.pos_serv
        return np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [-x, -y, -z, 1]
        ])

    def position(self):
        if self.mode == 0:
            return self.pos_serv
        elif self.mode == 1:
            return self.direction.T @ self.pos_serv
