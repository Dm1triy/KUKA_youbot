import numpy as np
from math import hypot, cos, sin


class Bug:
    def __init__(self, start_point=None,
                 end_point=None,
                 bin_map=None):
        self.start_point = start_point
        self.end_point = end_point
        # True - obstacle, False - free
        self.bool_map = bin_map

        # created nodes [x, y]
        self.nodes = np.array([self.start_point]).astype(np.uint32)
        # map with bool_map shape, -1 = not visited, 0 = created
        self.node_map = np.ones(self.bool_map.shape).astype(np.int16)*-1
        self.node_map[tuple(self.start_point)] = 0
        self.graph = {}
        # path is working without children
        # parent is index (node_num)
        #               [parent, [children], cost]
        self.graph[0] = [None, [], 0]
        # node index counter
        self.node_num = 1

        self.dir_line = self.direction_line()
        self.unit_vec = self.unit_vector()
        self.last_obs = [np.inf, np.inf]

        # 61x91
        map_shape = self.bool_map.shape
        # 60x90
        self.map_shape = (map_shape - np.ones(len(map_shape))).astype(np.uint32)
        self.dist_reached = False
        self.end_node = -1
        self.path = []
        self.graph_printed = False
        self.explore_obstacle = False

        if not start_point.any():
            print("No start point")
        assert start_point.any()
        if not end_point.any():
            print("No end point")
        assert end_point.any()
        if not bin_map.any():
            print("No bin_map")
        assert bin_map.any()

    def step(self):
        if self.nodes[-1].all() == self.end_point.all():
            self.dist_reached = True
            return
        if self.explore_obstacle:
            self.explore_obs()
        else:
            self.move_forward()

    def get_path(self):
        node_num = self.end_node
        self.path = []
        while node_num != 0:
            self.path.append(self.nodes[node_num])
            node_num = self.graph[node_num][0]
        self.path.append(self.nodes[node_num])
        if not self.graph_printed:
            self.graph_printed = True

    def explore_obs(self):
        curr_pos = self.nodes[-1]
        obs_vec = [self.last_obs[0] - curr_pos[0], self.last_obs[1] - curr_pos[1]]
        for i in range(8):
            rot_m = self.get_rotation_matrix(i)
            new_x, new_y = np.dot(rot_m, obs_vec)
            if not self.is_obstacle(new_x, new_y):
                self.nodes = np.append(self.nodes, [[new_x, new_y]], axis=0)
                self.node_map[(new_x, new_y)] = 0
                self.graph[self.node_num + 1] = [self.node_num, [], 0]
                self.node_num += 1
                self.last_obs = np.dot(self.get_rotation_matrix(i-1), obs_vec)
                if self.dist_from_line(new_x, new_y) < 0.05:
                    self.explore_obstacle = False

    def move_forward(self):
        new_x, new_y = np.round(self.nodes[-1] + self.unit_vec)
        if not self.is_obstacle(new_x, new_y):
            self.nodes = np.append(self.nodes, [[new_x, new_y]], axis=0)
            self.node_map[(new_x, new_y)] = 0
            self.graph[self.node_num+1] = [self.node_num, [], 0]
            self.node_num += 1
        else:
            self.last_obs = [new_x, new_y]
            self.explore_obstacle = True

    @staticmethod
    def get_rotation_matrix(i):
        theta = np.deg2rad(i * 45)
        return np.array([[cos(theta), sin(theta)], [-sin(theta), cos(theta)]])

    def is_obstacle(self, x, y):
        return self.bool_map[x][y]

    def direction_line(self):
        # (x - start_x)/(end_x - start_x) = (y - start_y)/(end_y - start_y)
        # directional vector = {end_x - start_x, end_y - start_y} = {dir_x, dir_y}
        # dir_y * x - dir_x * y - start_x * dir_y + start_y * dir_x = 0
        a = self.end_point[1] - self.start_point[1]   # dir_y
        b = self.start_point[0] - self.end_point[0]   # -dir_x
        # -start_x * dir_y + start_y * dir_x
        c = - self.start_point[0] * a - self.start_point[1] * b
        # ax + by + c = 0
        return [a, b, c]

    def unit_vector(self):
        return np.array([-self.dir_line[1], self.dir_line[0]])/hypot(self.dir_line[0], self.dir_line[1])

    def dist_from_line(self, x, y):
        return (self.dir_line[0] * x + self.dir_line[1] * y + self.dir_line[2]) \
               /hypot(self.dir_line[0], self.dir_line[1])