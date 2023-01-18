import numpy as np
import threading as thr
import cv2
import pygame as pg
import scipy

from pathfinding.RRT import RRT
from Pygame_GUI.Screen import Screen
from Pygame_GUI.Space3D.Space3D import Space3D


class TimeRrtGui:
    def __init__(self, width, height):
        self.width, self.height = width, height
        self.screen = Screen(self.width, self.height)
        self.space_3d = Space3D(self.screen, x=0, y=0, width=1, height=1)
        self.old_pressed_keys = []
        self.old_mouse_pos = [0, 0]
        self.space_3d.add_object([(-1, -1, -1, 1), (1, -1, -1, 1), (-1, 1, -1, 1), (-1, -1, 1, 1),
                                  (1, 1, -1, 1), (1, -1, 1, 1),
                                  (-1, 1, 1, 1), (1, 1, 1, 1)],
                                 [[0, 1, 4, 2], [3, 5, 7, 6], [0, 3, 5, 1], [6, 7, 4, 2], [1, 5, 7, 4], [0, 3, 6, 2]])

    def update_keys(self):
        presed_keys = self.screen.pressed_keys[:]
        transition = np.array([0, 0, 0, 0])
        if pg.K_a in presed_keys:
            transition[0] = -1
        if pg.K_d in presed_keys:
            transition[0] = 1
        if pg.K_w in presed_keys:
            transition[2] = 1
        if pg.K_s in presed_keys:
            transition[2] = -1
        if pg.K_q in presed_keys:
            transition[1] = 1
        if pg.K_e in presed_keys:
            transition[1] = -1

        if self.old_pressed_keys != presed_keys:
            if self.space_3d.camera.mode == 0:
                self.space_3d.camera.control(transition=transition)
            self.old_pressed_keys = presed_keys

    def run(self):
        while self.screen.running:
            self.update_keys()
            self.screen.step()


trgui = TimeRrtGui(1000, 1000)
trgui.run()
