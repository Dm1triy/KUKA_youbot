import numpy as np
import math

class Dijkstra:
    def __init__(self, start_point=None,
                 end_point=None,
                 bin_map=None):
        self.start_point = start_point
        self.end_point = end_point
        # True - obstacle, False - free
        self.bool_map = bin_map

        # visited nodes [x, y]
        self.nodes = np.array([self.start_point]).astype(np.uint32)
        # map with bool_map shape, -1 = not visited, 0 = visited
        self.node_map = np.ones(self.bool_map.shape).astype(np.int16)*-1
        self.node_map[tuple(self.start_point)] = 0
        self.graph = {}
        # path is working without children
        # parent is index (node_num)
        #               [parent, [children], cost]
        self.graph[0] = [None, [], 0]
        # node index counter
        self.node_num = 1

        # 61x91
        map_shape = self.bool_map.shape
        # 60x90
        self.map_shape = (map_shape - np.ones(len(map_shape))).astype(np.uint32)
        self.dist_reached = False
        self.end_node = -1
        self.path = []
        self.graph_printed = False

        self.motion = self.get_motion_model()

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
        for move_x, move_y, move_cost in self.motion:
            if not self.is_obstacle():

    def is_obstacle(self, x, y):
        return self.bool_map[y][x]


    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion


if __name__ == "__main__":
    pass