import numpy as np
import pygame as pg
from Pygame_GUI.Space3D.matrix_functions import *
from Pygame_GUI.Space3D.constants import *


class Camera:
    def __init__(self, render, position):
        self.render = render
        self.init_pos = position

        self.transition = np.array([0, 0, 0, 0])
        self.rotation = [0, 0]
        self.h_fov = math.pi / 3
        self.v_fov = self.h_fov * (render.height / render.width)
        self.near_plane = NEAR_PLANE
        self.far_plane = FAR_PLANE
        self.moving_speed = 0.3
        self.zoom_speed = 3
        self.rotation_speed = 0.011
        self.mode = 1
        self.direction = np.eye(4)

        # service variables
        self.pos_serv = np.array([*position, 1.0])
        self.pos_serv_1 = np.array([*position, 1.0])
        self.camera_matrix_0_serv = np.eye(4)
        self.camera_matrix_1_serv = np.eye(4)

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
            self.transition = transition * self.moving_speed

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

        # make matrix
        x, y, z, w = self.pos_serv
        translate_matrix = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [-x, -y, -z, 1]
        ])
        dir_t = self.direction.T
        if self.mode == 0:
            self.camera_matrix_0_serv = translate_matrix @ dir_t
        elif self.mode == 1:
            self.camera_matrix_1_serv = dir_t @ translate_matrix
            self.pos_serv_1 = dir_t @ self.pos_serv

    def camera_matrix(self):
        if self.mode == 0:
            return self.camera_matrix_0_serv
        elif self.mode == 1:
            return self.camera_matrix_1_serv

    def position(self):
        if self.mode == 0:
            return self.pos_serv
        elif self.mode == 1:
            return self.pos_serv_1
