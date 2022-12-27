import threading as thr

import cv2
import numpy as np
import pygame as pg
import scipy

from KUKA import KUKA
from Pygame_GUI.Screen import Screen
from RRT import RRT
from map_plotter import MapPlotter


class RRT_sim:
    def __init__(self, plotter=None, robot=None):
        self.robot = robot
        self.plotter = plotter
        self.screen_size = 900

        self.discrete = 20
        self.robot_radius = 17
        self.screen_obj = Screen(self.screen_size, self.screen_size)
        self.screen = self.screen_obj.screen
        self.move_speed_val = 0.5
        self.last_checked_pressed_keys = []
        self.step = False
        self.flow = False
        self.drive = False
        self.new_map = False
        self.end_point = np.array(False)
        self.start_point = np.array(False)
        self.nav_map = []
        self.map_arr = []
        self.manual_map()

        self.rrt_state = 0

    # make map
    def manual_map(self):
        self.map_shape = (60, 90)
        self.map_k = self.screen_size // max(self.map_shape[0], self.map_shape[1])
        self.map_arr = np.array([[[255, 255, 255]] * (self.map_shape[1] + 1)] * (self.map_shape[0] + 1)).astype(
            np.uint8)
        obstacle1 = [[0.81, 1.86], [0.73, 1.81], [1.31, 0.87], [1.42, 0.92]]
        obstacle1_conv = np.array(list(map(self.m_to_arr, obstacle1)), np.int32)
        obstacle2 = [[3.83, 1.87], [3.74, 1.93], [3.06, 0.94], [3.16, 0.87]]
        obstacle2_conv = np.array(list(map(self.m_to_arr, obstacle2)), np.int32)
        obstacle3 = [[2.07, 2.32], [1.81, 2.32], [1.81, 1.96], [2.07, 1.96]]
        obstacle3_conv = np.array(list(map(self.m_to_arr, obstacle3)), np.int32)

        cv2.rectangle(self.map_arr, (0, 0), (self.map_shape[1], self.map_shape[0]), (0, 0, 0), 1)
        cv2.fillPoly(self.map_arr, pts=[obstacle1_conv, obstacle2_conv, obstacle3_conv], color=(0, 0, 0))

        nav_map = []
        for i in self.map_arr:
            for j in i:
                if j[0] == 255:
                    nav_map.append(0)
                else:
                    nav_map.append(1)
        self.nav_map = np.array(nav_map)
        self.nav_map = self.nav_map.reshape(len(self.map_arr), len(self.map_arr[0]))

    def grab_map(self):
        try:
            del self.rrt
        except:
            pass
        self.map_arr = np.copy(self.plotter.map_background)
        self.nav_map = np.copy(self.plotter.map_arr)
        self.map_shape = self.nav_map.shape
        self.map_k = self.screen_size // max(self.map_shape[0], self.map_shape[1])
        self.start_point = np.array(self.plotter.scale_to_arr(*self.plotter.pos[:-1]))
        self.apply_robot_radius_to_map()

    def m_to_arr(self, coords):
        x, y = coords
        return [x * self.discrete, y * self.discrete]

    def arr_to_m(self, coords):
        x, y = coords
        return [x / self.discrete, y / self.discrete]

    def screen_to_arr(self, coords):
        x, y = coords
        return [x // self.map_k, y // self.map_k]

    def update_keys(self):
        pressed_keys = self.screen_obj.pressed_keys[:]
        move_speed = [0, 0, 0]
        fov = 0
        if pg.K_w in pressed_keys:
            fov += 1
            self.drive = False
        if pg.K_s in pressed_keys:
            fov -= 1
            self.drive = False
        move_speed[0] = fov * self.move_speed_val

        rot = 0
        if pg.K_a in pressed_keys:
            rot += 1
            self.drive = False
        if pg.K_d in pressed_keys:
            rot -= 1
            self.drive = False
        move_speed[2] = rot * self.move_speed_val

        side = 0
        if pg.K_q in pressed_keys:
            side += 1
            self.drive = False
        if pg.K_e in pressed_keys:
            side -= 1
            self.drive = False
        move_speed[1] = side * self.move_speed_val

        if pg.K_z in pressed_keys:
            self.step = True
            self.flow = False
        elif pg.K_x in pressed_keys:
            self.flow = True
            self.step = True
        if pg.K_g in pressed_keys:
            self.new_map = True
        if pg.K_v in pressed_keys:
            self.drive = True
        if pg.K_c in pressed_keys:
            self.drive = False
        if self.last_checked_pressed_keys != pressed_keys:
            if not self.drive:
                self.robot.move_base(*move_speed)
                self.robot.going_to_target_pos = False
                self.last_checked_pressed_keys = pressed_keys[:]

        if self.screen_obj.mouse_clicked:
            self.screen_obj.mouse_clicked = False
            if not self.start_point.any():
                self.start_point = np.array(self.screen_to_arr(self.screen_obj.mouse_pos))
                print("start point:", self.start_point)
            elif not self.end_point.any():
                self.end_point = np.array(self.screen_to_arr(self.screen_obj.mouse_pos))
                print("end point:", self.end_point)

    def draw_map(self):

        map_img = pg.transform.scale(pg.surfarray.make_surface(self.map_arr),
                                     (self.map_shape[0] * self.map_k, self.map_shape[1] * self.map_k))
        self.screen.blit(map_img, (0, 0))
        if self.start_point.any():
            pg.draw.circle(self.screen, (255, 0, 0), list(map(lambda x: x * self.map_k, self.start_point)), 5)
        if self.end_point.any():
            pg.draw.line(self.screen, (0, 102, 51), [list(map(lambda x: x * self.map_k, self.end_point))[0] - 10,
                                                     list(map(lambda x: x * self.map_k, self.end_point))[1] - 10],
                         [list(map(lambda x: x * self.map_k, self.end_point))[0] + 10,
                          list(map(lambda x: x * self.map_k, self.end_point))[1] + 10], 5)
            pg.draw.line(self.screen, (0, 102, 51), [list(map(lambda x: x * self.map_k, self.end_point))[0] + 10,
                                                     list(map(lambda x: x * self.map_k, self.end_point))[1] - 10],
                         [list(map(lambda x: x * self.map_k, self.end_point))[0] - 10,
                          list(map(lambda x: x * self.map_k, self.end_point))[1] + 10], 5)

    def draw_curr_pos(self):
        pg.draw.circle(self.screen, (255, 0, 102),
                       list(map(lambda x: x * self.map_k, self.plotter.scale_to_arr(*self.plotter.pos[:-1]))), 10)

    def draw_nodes(self):
        for j in range(self.rrt.nodes.shape[0]):
            i = self.rrt.nodes[j]
            pg.draw.circle(self.screen, (0, 0, 255), list(map(lambda x: x * self.map_k, i)), 5)
        pg.draw.circle(self.screen, (255, 0, 0), list(map(lambda x: x * self.map_k, self.start_point)), 5)

    def draw_edges(self):
        for i in range(1, self.rrt.node_num):
            n = self.rrt.graph[i][0]
            pg.draw.aaline(self.screen, (255, 0, 255), list(map(lambda x: x * self.map_k, self.rrt.nodes[i])),
                           list(map(lambda x: x * self.map_k, self.rrt.nodes[n])))
        pg.draw.circle(self.screen, (255, 0, 255), list(map(lambda x: x * self.map_k, self.rrt.random_point)),
                       5)

    def draw_path(self):
        pg.draw.lines(self.screen, (255, 0, 0), False,
                      [[*i] for i in list(map(lambda x: x * self.map_k, self.rrt.path))], 5)

    def apply_robot_radius_to_map(self):
        n_mask = scipy.ndimage.generate_binary_structure(len(self.map_shape), 3)
        neighborhood = np.zeros((self.robot_radius, self.robot_radius))
        neighborhood[self.robot_radius // 2][self.robot_radius // 2] = 1
        neighborhood = scipy.ndimage.binary_dilation(neighborhood, structure=n_mask).astype(n_mask.dtype)
        for i in range(int(self.robot_radius // 2 / 3)):
            neighborhood = scipy.ndimage.binary_dilation(neighborhood, structure=neighborhood).astype(n_mask.dtype)
        bool_map = np.array(self.nav_map).astype(np.uint8) == 0
        bool_map = scipy.ndimage.binary_erosion(bool_map, structure=neighborhood, border_value=1)
        self.bool_map = bool_map == False

    def init_rrt(self):
        self.rrt = RRT(start_point=np.array(self.start_point), end_point=np.array(self.end_point),
                       bin_map=self.bool_map)
        self.rrt.step()

    def main_thr(self):
        while self.screen_obj.running:
            self.update_keys()
            self.draw_map()
            self.screen_obj.step()
            if not self.drive:
                self.robot.going_to_target_pos = False
            if self.rrt_state >= 0:
                self.draw_nodes()
                self.draw_edges()
            if self.rrt_state == 2:
                self.rrt.get_path()
                self.draw_path()
                self.draw_curr_pos()
            self.screen_obj.step()
        self.screen_obj.end()

    def pathfinding_rrt(self):
        self.apply_robot_radius_to_map()
        self.init_rrt()
        self.rrt_state = 1
        while self.screen_obj.running:
            if self.step:
                self.rrt.step()
            if self.rrt.dist_reached:
                self.rrt_state = 2
                break
            if not self.flow:
                self.step = False

    def travel_path(self):
        curr_point = 1
        while curr_point < len(self.rrt.path) + 2:
            if self.drive:
                if curr_point < len(self.rrt.path) + 1:
                    prec = 0.005
                    k = 1
                    goal = self.plotter.scale_to_m(*self.rrt.path[-curr_point])
                else:
                    prec = 0.005
                    k = 1

                if self.drive and not self.robot.going_to_target_pos:
                    self.robot.go_to(*goal, prec=prec, k=k)
                    curr_point += 1

    def start_old(self):

        while self.running:
            curr_point = 1
            while not self.new_map:
                self.update_keys()
                self.draw_map()
                self.screen_obj.step()
                if self.end_point.any():
                    self.apply_robot_radius_to_map()
                    break
            while not self.new_map:
                self.update_keys()
                if not self.drive:
                    self.robot.going_to_target_pos = False
                    goal = None
                self.draw_map()
                if self.step:
                    self.rrt.step()
                self.draw_nodes()
                self.draw_edges()
                if self.rrt.dist_reached:
                    self.rrt.get_path()
                    self.draw_path()
                    self.draw_curr_pos()
                    if self.drive and curr_point < len(self.rrt.path) + 2:

                        if curr_point < len(self.rrt.path) + 1:
                            prec = 0.005
                            k = 1
                            goal = self.plotter.scale_to_m(*self.rrt.path[-curr_point])
                        else:
                            prec = 0.005
                            k = 1

                        if self.drive and not self.robot.going_to_target_pos:
                            self.robot.go_to(*goal, prec=prec, k=k)
                            curr_point += 1
                    else:
                        self.drive = False
                self.screen_obj.step()
                if not self.flow:
                    self.step = False
            self.grab_map()
            self.new_map = False
            self.end_point = np.array(False)
        pg.quit()


robot = KUKA('192.168.88.21', ros=False, offline=False, read_depth=False, camera_enable=False)

rrt_sim = RRT_sim(None, robot)
rrt_sim.start()
