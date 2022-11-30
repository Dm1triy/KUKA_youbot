import numpy as np
import math


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

    def explore_obstacle(self):


    def move_forward(self):
        new_x, new_y = np.round(self.nodes[-1] + self.unit_vec)
        if not self.is_obstacle(new_x, new_y):
            self.nodes = np.append(self.nodes, [[new_x, new_y]], axis=0)
            self.node_map[(new_x, new_y)] = 0
            self.graph[self.node_num+1] = [self.node_num, [], 0]
            self.node_num += 1
        else:
            self.explore_obstacle = True

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
        return np.array([-self.dir_line[1], self.dir_line[0]])/math.hypot(self.dir_line[0], self.dir_line[1])

    def dist_from_line(self, x, y):
        return (self.dir_line[0] * x + self.dir_line[1] * y + self.dir_line[2]) \
               /math.hypot(self.dir_line[0], self.dir_line[1])