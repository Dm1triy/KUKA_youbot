import numpy as np
import cv2 as cv
import skimage
import time
import threading as thr
from acceleration.client import Client


class ForwardKinematics:
    def __init__(self, robot, debug=False):
        # arm_base is manipulator pos if x=0, y=0 are the coords of the robot's center and z=0 is ground
        self.arm_base = 0, 0.18, 0.3
        self.link_len = [0.16, 0.14, 0.11]  # m2_len, m3_len, m4_len in meters
        self.robot = robot
        self.debug = debug

        if not self.debug:
            self.angles = self.robot.arm  # 5 angles of 1-5 links in radiansу
        else:
            self.angles = self.read_angs_from_log()

    def update_arm(self):
        self.angles = self.robot.arm

    @staticmethod
    def read_angs_from_log(path='debug/armpos.txt'):
        path = path
        f = open(path, "r")
        angles = list(map(float, f.read().split(' ')))
        f.close()
        return angles

    def get_cam_pos(self):
        """
        calculates the position of the camera relative to the center of the robot
        :return:
        """
        # clockwise rotation is described by the following matrix
        #   |cos(x)  -sin(x) ...|                                       |cos(x)  sin(x) ...|
        #   |sin(x) cos(x)      | and counterclockwise rotation by the  |-sin(x) cos(x)    |
        # theta is positive when rotation is clockwise
        theta = self.angles[0]
        theta = np.radians(theta)
        # rotation of the 1 link around z-axis
        affine_mat = np.array([[np.cos(theta), -np.sin(theta), 0, 0],
                               [np.sin(theta), np.cos(theta),  0, 0],
                               [0,             0,              1, self.link_len[0]],
                               [0,             0,              0, 1]])
        for i in range(len(self.link_len)-1):
            theta = np.radians(self.angles[i+1])

            # rotation around x-axis and z-offset
            link_mat = np.array([[1, 0,             0,              0],
                                 [0, np.cos(theta), -np.sin(theta), 0],
                                 [0, np.sin(theta), np.cos(theta),  self.link_len[i+1]],
                                 [0, 0,             0,              1]])
            affine_mat = np.matmul(affine_mat, link_mat)

        pos_mat = self.arm_base + (1,)  # [x, y, z, 1]
        pos_mat = np.array(pos_mat).reshape(-1, 1)  # transpose
        cam_pos = np.matmul(affine_mat, pos_mat)[:-1].reshape(-1)   # [x_new, y_new, z_new]
        ang = abs(90 + sum(self.angles[1:4]))
        cam_pos = np.append(cam_pos, ang)
        return cam_pos  # x, y, z, theta


class SurfaceMap:
    def __init__(self, robot, client, debug=False):
        self.robot = robot
        self.client = client        # for accelerometer vel receiving

        self.vel_counter = None
        self.accel_velocity = None
        self.odom_velocity = None

        self.cell_size = 0.05   # meters
        self.map_width = 501
        self.map_height = 501
        self.surface_map = np.array([[[100, 100, 100]] * self.map_width] * self.map_height, dtype=np.uint8)
        self.start_x, self.start_y = self.map_width//2, self.map_height//2

        # camera params
        self.vert_FoV = np.radians(40)
        self.horiz_FoV = np.radians(50)
        self.focal_len = 1.93   # mm

        # For forward kinematics
        self.link_len = [0.16, 0.14, 0.11]  # m2_len, m3_len, m4_len in meters
        # arm_base is manipulator pos if x=0, y=0 are the coords of the robot's center and z=0 is ground
        self.arm_base = 0, 0.18, 0.3

        if robot:
            # robot values in the moment
            self.floor_img = robot.camera_BGR()
            self.angles = self.robot.arm
            self.robot_pos, _ = self.robot.lidar
            self.running = True
            vel_thr = thr.Thread(target=self.update_velocity, args=())
            vel_thr.start()
        else:
            self.floor_img = self.get_img_from_log()
            self.angles = self.get_angs_from_log()
            self.robot_pos = [2.0, -0.5, np.radians(30)]

        cam_coords = self.forward_kinematics()
        self.arm_pos = cam_coords[:3]
        self.arm_angle = np.radians(cam_coords[-1])

    def update_velocity(self):
        while self.running:
            self.accel_velocity = self.client.get_velocity()    # absolute velocity
            while self.vel_counter == self.robot.odom_speed_counter:
                time.sleep(0.05)
            self.vel_counter = self.robot.odom_speed_counter
            self.odom_velocity = self.robot.odom_speed_data     # Vx, Vy, rotation
            abs_vel_odom = np.linalg.norm([self.odom_velocity[0], self.odom_velocity[1]])
            print(f"Accel_vel = {self.accel_velocity},      Odom_vel = {abs_vel_odom}")

    def create_surface_map(self):
        while self.running:
            self.running = cv.waitKey(40) != 27
            self.floor_img = self.robot.camera_BGR()
            self.angles = self.robot.arm
            self.robot_pos, _ = self.robot.lidar

            cam_coords = self.forward_kinematics()
            self.arm_pos = cam_coords[:3]
            self.arm_angle = np.radians(cam_coords[-1])

            if self.robot_pos and sum(self.angles):
                transformed_img = self.perspective_transform()
                floor_dims = self.get_floor_sizes()
                local_map = self.map_from_img(transformed_img, floor_dims)
                self.update_surf_map(local_map)
                img = cv.resize(self.surface_map, dsize=(1000, 1000), interpolation=cv.INTER_NEAREST)
                cv.imshow("map", img)
            time.sleep(1)

    def debug(self):
        def display_img(img, size=(500, 500)):
            scale_width = size[0]//img.shape[0]
            scale_height = size[1]//img.shape[1]
            img = cv.resize(img, size, interpolation=cv.INTER_NEAREST)
            cv.imshow('Floor', img)
            cv.waitKey(0)
            cv.destroyAllWindows()

        display_img(self.floor_img)
        transformed_img = self.perspective_transform()
        display_img(transformed_img)
        floor_dims = self.get_floor_sizes()
        local_map = self.map_from_img(transformed_img, floor_dims)
        self.update_surf_map(local_map)
        display_img(self.surface_map)

    def forward_kinematics(self):
        """
        calculates the position of the camera relative to the center of the robot
        :return:
        """
        # clockwise rotation is described by the following matrix
        #   |cos(x)  -sin(x) ...|                                       |cos(x)  sin(x) ...|
        #   |sin(x) cos(x)      | and counterclockwise rotation by the  |-sin(x) cos(x)    |
        # theta is positive when rotation is clockwise
        theta = self.angles[0]
        theta = np.radians(theta)
        # rotation of the 1 link around z-axis
        affine_mat = np.array([[np.cos(theta), -np.sin(theta), 0, 0],
                               [np.sin(theta), np.cos(theta),  0, 0],
                               [0,             0,              1, self.link_len[0]],
                               [0,             0,              0, 1]])
        for i in range(len(self.link_len)-1):
            theta = np.radians(self.angles[i+1])

            # rotation around x-axis and z-offset
            link_mat = np.array([[1, 0,             0,              0],
                                 [0, np.cos(theta), -np.sin(theta), 0],
                                 [0, np.sin(theta), np.cos(theta),  self.link_len[i+1]],
                                 [0, 0,             0,              1]])
            affine_mat = np.matmul(affine_mat, link_mat)

        pos_mat = self.arm_base + (1,)  # [x, y, z, 1]
        pos_mat = np.array(pos_mat).reshape(-1, 1)  # transpose
        cam_pos = np.matmul(affine_mat, pos_mat)[:-1].reshape(-1)   # [x_new, y_new, z_new]
        ang = abs(90 + sum(self.angles[1:4]))
        cam_pos = np.append(cam_pos, ang)
        return cam_pos  # x, y, z, theta

    def perspective_transform(self):
        step = self.floor_img.shape[0] * np.tan(self.arm_angle)
        shift = 40  # it is related to camera tilt
        # old dots
        pt_a = [step, 0]
        pt_b = [self.floor_img.shape[1] - step + shift, 0]
        pt_c = [self.floor_img.shape[1] - 1, self.floor_img.shape[0] - 1]
        pt_d = [0, self.floor_img.shape[0] - 1]
        # save distances
        width = int(self.floor_img.shape[1])
        height = int(np.sqrt(pt_a[0] ** 2 + pt_d[1] ** 2))

        old_pts = np.float32([pt_a, pt_b, pt_c, pt_d])
        new_pts = np.float32([[0, 0], [width - 1, 0], [width - 1, height - 1], [0, height - 1]])
        M = cv.getPerspectiveTransform(old_pts, new_pts)

        transformed_img = cv.warpPerspective(self.floor_img, M, (width, height), flags=cv.INTER_LINEAR)

        return transformed_img

    def get_floor_sizes(self):
        """
        calculates the sizes of the visible part of the floor
        :return: width_x and width_y of the floor in meters
        """
        cam_height = self.arm_pos[-1]
        theta = self.arm_angle

        top = cam_height * np.tan(np.pi/2 - theta + self.vert_FoV/2)
        bottom = cam_height * np.tan(np.pi/2 - theta - self.vert_FoV/2)
        vert_dist = top - bottom
        horiz_dist = 2 * cam_height * np.tan(self.horiz_FoV/2)/np.cos(np.pi/2 - theta - self.vert_FoV/2)
        return horiz_dist, vert_dist

    def map_from_img(self, img, map_size):
        floor_size = map_size
        map_width, map_height = int(floor_size[0] / self.cell_size), int(floor_size[1] / self.cell_size)
        scale_x, scale_y = img.shape[0] // map_width, img.shape[1] // map_height
        local_map = skimage.measure.block_reduce(img, (scale_y, scale_x, 1), np.max)
        return local_map

    def update_surf_map(self, local_map):
        from itertools import product

        robot_x, robot_y = self.pos_to_cell(self.robot_pos[0], self.robot_pos[1])
        robot_ang = self.robot_pos[2]
        arm_len = np.linalg.norm([self.arm_pos[0], self.arm_pos[1]])
        arm_x, arm_y = robot_x + (arm_len / self.cell_size) * np.cos(self.arm_angle), \
                       robot_y + (arm_len / self.cell_size) * np.sin(self.arm_angle)

        dist_from_cam = self.arm_pos[-1] * np.tan(np.pi/2 - self.arm_angle - self.vert_FoV/2)
        x_from_cam = dist_from_cam / self.cell_size * np.cos(robot_ang)
        y_from_cam = dist_from_cam / self.cell_size * np.sin(robot_ang)

        img_edge_x = int(arm_x + x_from_cam)
        img_edge_y = int(arm_y + y_from_cam)

        iter_x, iter_y = range(local_map.shape[0]), range(local_map.shape[1])
        for i, j in product(iter_x, iter_y):
            affine_mat = np.array([[np.cos(robot_ang), -np.sin(robot_ang)],
                                   [np.sin(robot_ang), np.cos(robot_ang)]])
            new_i, new_j = np.matmul(affine_mat, np.array([i, j]))
            diff_j, diff_i = (local_map.shape[1]//2 - 1) * np.cos(robot_ang), \
                             (local_map.shape[1]//2) * np.sin(robot_ang)
            new_i = np.around(img_edge_x + new_i + diff_i).astype(int)
            new_j = np.around(img_edge_y + new_j - diff_j).astype(int)
            self.surface_map[new_j, new_i] = local_map[-i, j]

        self.surface_map = cv.medianBlur(self.surface_map, 3)

        self.surface_map[self.start_y, self.start_x] = [0, 0, 0]
        self.surface_map[robot_y, robot_x] = [255, 0, 0]
        self.surface_map = cv.circle(self.surface_map, (robot_x, robot_y), 5, [255, 0, 0], 5)
        # self.surface_map[arm_y, arm_x] = [255, 0, 0]
        self.surface_map[img_edge_y, img_edge_x] = [255, 0, 0]
        # Putting text

    def pos_to_cell(self, x, y):
        return int(self.start_x + x / self.cell_size), int(self.start_y - y / self.cell_size)

    @staticmethod
    def get_img_from_log(path='/home/kpu/dev/__/KUKA_youbot0/debug/floor.jpg'):
        img = cv.imread(path)
        img = cv.cvtColor(img, cv.COLOR_BGR2RGB)
        return img

    @staticmethod
    def get_angs_from_log(path='/home/kpu/dev/__/KUKA_youbot0/debug/armpos.txt'):
        path = path
        f = open(path, "r")
        angles = list(map(float, f.read().split(' ')))
        f.close()
        return angles


if __name__ == '__main__':
    adapt = SurfaceMap(None, debug=True)
    adapt.debug()