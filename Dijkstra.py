import numpy as np
import math

class Dijkstra:
    def __init__(self, start_point=None,
                 end_point=None,
                 bin_map=None):
        # format: x, y
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

        # costs of not visited nodes
        self.not_visited_nodes = dict()
        self.not_visited_nodes[0] = 0

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
        best_node_index = self.best_node()
        new_x, new_y = self.nodes[best_node_index]
        parent_cost = self.graph[best_node_index][2]
        for move_x, move_y, move_cost in self.motion:
            new_x += move_x
            new_y += move_y
            if not self.is_obstacle(new_x, new_y):
                if self.node_map[new_x][new_y] != 0:
                    self.graph[self.node_num] = [best_node_index, [], parent_cost + move_cost]
                    self.not_visited_nodes[self.node_num] = parent_cost + move_cost
                    self.node_map[new_x][new_y] = 0
                    self.nodes = np.append(self.nodes, [[new_x, new_y]], axis=0)
                    self.node_num += 1
                else:
                    index = np.where((self.nodes == (new_x, new_y)).all(axis=1))[0][0]
                    old_cost = self.graph[index][2]
                    if old_cost > parent_cost + move_cost:
                        self.graph[index] = [best_node_index, [], parent_cost + move_cost]
                        if index in self.not_visited_nodes:
                            self.not_visited_nodes[index] = parent_cost + move_cost
                if (new_x, new_y) == tuple(self.end_point):
                    self.end_node = self.node_num-1
                    self.dist_reached = True

    def get_path(self):
        node_num = self.end_node
        self.path = []
        while node_num != 0:
            self.path.append(self.nodes[node_num])
            node_num = self.graph[node_num][0]
        self.path.append(self.nodes[node_num])
        if not self.graph_printed:
            self.graph_printed = True

    def best_node(self):
        i = min(self.not_visited_nodes, key=self.not_visited_nodes.get)
        del self.not_visited_nodes[i]
        return i

    def is_obstacle(self, x, y):
        return self.bool_map[x][y]

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