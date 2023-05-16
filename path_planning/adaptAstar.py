import numpy as np
import math


class Astar:
    def __init__(self, start_point=None,
                 end_point=None, bin_map=None):
        self.start_point = start_point
        self.end_point = end_point
        self.bool_map = bin_map

        # created nodes [x, y]
        self.nodes = np.array([self.start_point]).astype(np.uint32)
        # map with bool_map shape, -1 = unvisited, 0 = created
        self.node_map = np.ones(self.bool_map.shape).astype(np.int16)*-1
        self.node_map[tuple(self.start_point)] = 0
        self.graph = {0: [None, self.heuristic(self.start_point)]}     # node_index: [parent_node, cost]
        self.node_num = 1

        # here nodes that have the shell of 8 neighboring nodes
        # which still haven't been saved yet
        self.nodes_queue = dict()
        self.nodes_queue[0] = self.heuristic(self.start_point)

        map_shape = self.bool_map.shape
        self.map_shape = (map_shape - np.ones(len(map_shape))).astype(np.uint32)    # even shape
        self.dist_reached = False
        self.end_node = -1
        self.path = []
        self.graph_printed = False  # ??

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
        deb = False
        best_node_index = self.best_node()
        x, y = self.nodes[best_node_index]
        parent_cost = self.graph[best_node_index][1]
        for move_x, move_y, move_cost in self.motion:
            new_x = x + move_x
            new_y = y + move_y
            h = self.heuristic((new_x, new_y))
            res_f = h + move_cost
            if not self.is_obstacle(new_x, new_y):
                if self.node_map[new_x][new_y] != 0:    # if unvisited
                    self.graph[self.node_num] = [best_node_index, parent_cost + res_f]
                    self.nodes_queue[self.node_num] = parent_cost + res_f   # add to applicants
                    self.node_map[new_x][new_y] = 0     # no longer unvisited
                    self.nodes = np.append(self.nodes, [[new_x, new_y]], axis=0)    # pos
                    self.node_num += 1
                else:   # if already visited
                    # where node == new_x and new_y; [0][0] for removing brackets
                    index = np.where((self.nodes == (new_x, new_y)).all(axis=1))[0][0]
                    old_cost = self.graph[index][1]
                    if old_cost > parent_cost + res_f:
                        self.graph[index] = [best_node_index, parent_cost + res_f]
                        if index in self.nodes_queue:
                            self.nodes_queue[index] = parent_cost + res_f
                if (new_x, new_y) == tuple(self.end_point):
                    self.end_node = self.node_num - 1
                    print(deb)
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
        return self.path

    def heuristic(self, point):
        return np.linalg.norm([self.end_point[0] - point[0], self.end_point[1]-point[1]])

    def best_node(self):
        i = min(self.nodes_queue, key=self.nodes_queue.get)
        del self.nodes_queue[i]
        return i

    def is_obstacle(self, x, y):
        return self.bool_map[y][x]

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, np.sqrt(2)],
                  [-1, 1, np.sqrt(2)],
                  [1, -1, np.sqrt(2)],
                  [1, 1, np.sqrt(2)]]
        return motion


if __name__ == "__main__":
    pass
