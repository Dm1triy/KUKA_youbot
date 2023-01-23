import numpy as np
import threading as thr
import cv2
import pygame as pg
import scipy

from pathfinding.RRT import RRT
from Pygame_GUI.Screen import Screen
from Pygame_GUI.Space3D.Space3D import Space3D
from Pygame_GUI.Objects import *


class TimeRrtGui:
    def __init__(self, width, height):
        self.width, self.height = width, height
        self.screen = Screen(self.width, self.height)
        self.space_3d = Space3D(self.screen, x=0, y=0, width=1, height=1)
        self.old_pressed_keys = []
        self.old_mouse_pos = [0, 0]
        self.space_3d.load_object_from_file("Pygame_GUI/Space3D/t_34_obj.obj")
        self.space_3d.add_object([(-1, -1, -1, 1), (1, -1, -1, 1), (-1, 1, -1, 1), (-1, -1, 1, 1),
                                  (1, 1, -1, 1), (1, -1, 1, 1),
                                  (-1, 1, 1, 1), (1, 1, 1, 1)],
                                 [[0, 1, 4, 2][::-1], [3, 6, 7, 5][::-1], [0, 3, 5, 1][::-1], [2, 4, 7, 6][::-1], [1, 5, 7, 4][::-1], [2, 6, 3, 0][::-1]])
        Button(self.screen, x=0.9, y=0.01, width=0.08, height=0.06, color=(150, 255, 170), func=self.change_cam_mode)


    def run(self):
        while self.screen.running:
            self.screen.step()

    def change_cam_mode(self, *args):
        print(123)
        self.space_3d.camera.mode = not self.space_3d.camera.mode
        self.space_3d.camera.reset()


trgui = TimeRrtGui(2000, 2000)
trgui.run()
