import time

import numpy as np
import threading as thr
import cv2
import pygame as pg
import scipy

from pathfinding.RRT import RRT
from KUKA.KUKA import KUKA
from Pygame_GUI.Screen import Screen
from pathfinding.SLAM import SLAM


class RRT_sim:
    def __init__(self, robot=None):
        self.robot = robot
        self.screen_size = 2000

        self.discrete = 30
        self.robot_radius = int(0.3*self.discrete+1)
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
        self.nav_map = np.array(False)
        self.map_arr = np.array(False)


        self.rrt_state = 0
        self.rrt_thr = thr.Thread(target=self.pathfinding_rrt, args=())
        self.curr_point = 1
        self.goal = np.array(False)

        self.slam = SLAM(self.robot)
        self.slam.no_slam = True
        self.slam.discrete = self.discrete
        self.slam_thr = thr.Thread(target=self.slam.run, args=())
        self.slam_thr.start()

        #self.manual_map()

    # make map
    def manual_map(self):
        self.map_shape = (300, 450)
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
        self.slam.bool_map = self.nav_map.reshape(len(self.map_arr), len(self.map_arr[0]))

    def m_to_arr(self, coords):
        x, y = coords
        return [x * self.discrete, y * self.discrete]

    def grab_map(self):
        # try:
        #    del self.rrt
        # except:
        #    pass
        self.nav_map = self.slam.bool_map
        self.map_shape = self.nav_map.shape
        self.map_k = self.screen_size / max(self.map_shape[0], self.map_shape[1])
        # self.start_point = np.array(self.slam.corr_pos)
        # map_arr = [[[255, 255, 255]] * self.map_shape[0]] * self.map_shape[1]
        # for i in range(self.map_shape[0]):
        #    for j in range(self.map_shape[1]):
        #        if self.nav_map[i][j] == 1:
        #            map_arr[i][j] = [0, 0, 0]
        # self.map_arr = np.array(map_arr)
        # self.apply_robot_radius_to_map()

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
            if not self.step:
                self.step = True
                self.flow = False
                self.rrt_thr = thr.Thread(target=self.pathfinding_rrt, args=())
                self.rrt_thr.start()
            else:
                self.flow = False
        elif pg.K_x in pressed_keys:
            if not self.step:
                self.flow = True
                self.step = True
                self.rrt_thr = thr.Thread(target=self.pathfinding_rrt, args=())
                self.rrt_thr.start()
        if pg.K_g in pressed_keys:
            self.new_map = True
        if pg.K_v in pressed_keys:
            if not self.drive:
                self.drive = True
                self.travel_path_thr = thr.Thread(target=self.travel_path, args=())
                self.travel_path_thr.start()


        if pg.K_c in pressed_keys:
            self.drive = False
        if self.last_checked_pressed_keys != pressed_keys:
            if not self.drive:
                self.robot.move_base(*move_speed)
                self.robot.going_to_target_pos = False
                self.last_checked_pressed_keys = pressed_keys[:]

        if self.screen_obj.mouse_state[1]:
            if not self.start_point.any():
                #self.start_point = np.array([self.screen_to_arr(self.screen_obj.mouse_pos)]).astype(int)
                self.start_point = (np.array([self.robot.increment[:2]])*self.discrete+np.array(self.map_shape).T/2).astype(int)
                print(self.robot.increment[:2])
                print("start point:", self.start_point)
            elif not self.end_point.any():
                self.end_point = np.array(self.screen_to_arr(self.screen_obj.mouse_pos)).astype(int)
                print("end point:", self.end_point)

    def draw_map(self):
        if self.nav_map.any():
            map_img = pg.transform.scale(pg.surfarray.make_surface((self.nav_map * -1 + 1) * 255),
                                         (self.map_shape[0] * self.map_k, self.map_shape[1] * self.map_k))
            self.screen.blit(map_img, (0, 0))
        if self.start_point.any():
            for sp in self.start_point:
                pg.draw.circle(self.screen, (255, 0, 0), list(map(lambda x: x * self.map_k, sp)), 5)
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
        if self.slam.no_slam:
            if self.robot.increment:

                half = self.map_shape[0] // 2
                pg.draw.circle(self.screen, (0, 0, 102), list(
                    map(lambda x: (half + x * self.discrete) * self.map_k, self.robot.increment[:2])), 10)
        else:
            if self.robot.increment_by_wheels:
                half = self.map_shape[0] // 2
                pg.draw.circle(self.screen, (0, 0, 102), list(
                    map(lambda x: (half + x * self.discrete) * self.map_k, self.robot.increment_by_wheels[:2])), 10)

    def draw_nodes(self):
        for j in range(self.rrt.nodes.shape[0]):
            i = self.rrt.nodes[j]
            pg.draw.circle(self.screen, (0, 0, 255), list(map(lambda x: x * self.map_k, i)), 5)
        #pg.draw.circle(self.screen, (255, 0, 0), list(map(lambda x: x * self.map_k, self.start_point)), 5)

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
        self.rrt = RRT(start_point=self.start_point, end_point=np.array(self.end_point),
                       bin_map=self.bool_map)
        self.rrt.step()

    def main_thr(self):
        while self.screen_obj.running:
            self.update_keys()
            self.grab_map()
            self.draw_map()
            self.draw_curr_pos()
            if self.rrt_state > 1:
                self.draw_nodes()
                self.draw_edges()
            if self.rrt_state == 3:
                self.rrt.get_path()
                self.draw_path()
            self.screen_obj.step()
        self.screen_obj.end()

    def pathfinding_rrt(self):
        if self.rrt_state == 0:
            self.rrt_state = 1
            self.apply_robot_radius_to_map()
            self.init_rrt()
            self.rrt.step()
            self.rrt_state = 2
        while self.screen_obj.running and self.step:
            if self.step:
                self.rrt.step()
            if self.rrt.dist_reached:
                self.rrt_state = 3
            if not self.flow:
                self.step = False

    def travel_path(self):
        print(self.rrt.path)
        print(self.curr_point, len(self.rrt.path))
        while self.curr_point < len(self.rrt.path) + 2 and self.drive and self.screen_obj.running:
            print(self.goal, self.robot.increment)
            if self.curr_point < len(self.rrt.path) + 1:
                prec = 0.005
                k = 1
                self.goal = np.array((self.rrt.path[-self.curr_point] - np.array(self.map_shape).T / 2) / self.discrete)

            else:
                prec = 0.5
                k = 3

            if self.drive and not self.robot.going_to_target_pos and self.goal.any():
                print(self.goal)
                self.robot.go_to(*self.goal, prec=prec, k=k)
                self.curr_point += 1

            time.sleep(0.2)
        self.robot.going_to_target_pos = False
        print("end drive")



robot = KUKA('192.168.88.21', ros=True, offline=False, read_depth=False, camera_enable=False, advanced=False)

rrt_sim = RRT_sim(robot)
rrt_sim.main_thr()
