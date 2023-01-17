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
        self.mouse_sense = 0.1

    def update_keys(self):
        presed_keys = self.screen.pressed_keys[:]
        transition = np.array([0, 0, 0, 1.0])
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

        mouse_pos = self.screen.mouse_pos[:]
        rotation = (self.old_mouse_pos[0] - mouse_pos[0]) * self.mouse_sense, (
                    self.old_mouse_pos[1] - mouse_pos[1]) * self.mouse_sense

        if self.old_pressed_keys != presed_keys:
            self.space_3d.camera.control(transition=transition)
            self.old_pressed_keys = presed_keys
        if self.screen.mouse_clicked == 1:
            self.space_3d.camera.control(rotation=rotation)
        self.old_mouse_pos = mouse_pos

    def run(self):
        while self.screen.running:
            self.update_keys()
            self.screen.step()


trgui = TimeRrtGui(1000, 1000)
trgui.run()
