import random
import math
import time

from pathfinding.vanilla_rrt.fast_math_for_rrt import *
from pathfinding.vanilla_rrt.rrt_graph import *

import threading as thr


class TreeRRT:
    def __init__(self, /,
                 start_point=None,
                 start_point_available=None,
                 end_point=np.array(False),
                 bin_map=None,
                 growth_factor=5,
                 e=0.04,
                 end_dist=6,
                 rrt_star_rad=5):
        self.end_point = end_point
        self.bool_map = bin_map

        self.graph = Graph(start_point=start_point, start_point_available=start_point_available, end_point=end_point)
        self.rand_list = [range(len(start_point) + len(end_point))]

        map_shape = self.bool_map.shape
        self.map_shape = (map_shape - np.ones(len(map_shape))).astype(np.int32)
        self.stuck = 0
        self.force_random = 0
        self.dist_reached = False
        self.end_node = -1
        self.path = []
        self.path_ind = []
        self.graph_printed = False
        self.random_point = None
        self.main_thr = thr.main_thread()
        self.opened_nodes = [x for x in self.graph.graph.keys()]
        self.closed_nodes = []

        if not start_point.any():
            print("No start point")
        assert start_point.any()
        # if not end_point.any():
        #    print("No end point")
        assert end_point.any()
        if not bin_map.any():
            print("No bin_map")
        assert bin_map.any()

        self.step_lock = thr.Lock()

        # settings and switches
        self.star = True
        self.open_closed_lists = True
        self.overpopulation_num = 7
        self.overpopulation_radius = 2
        self.growth_factor = growth_factor
        self.e = e
        self.end_dist = end_dist
        self.rrt_star_rad = rrt_star_rad
        # biases for random
        self.open_list_bias = 0.9

    def check_obstacle(self, point1, point2):
        info = np.zeros(len(point1) + 2).astype(np.float64)
        check_obstacle(point1, point2, self.bool_map, self.growth_factor, self.e, info)
        node, dist, reached = info[:3], info[3], info[4]
        node = node.astype(np.int32)
        reached = bool(reached)
        return node, dist, reached

    def generate_rand_point(self):
        return (np.random.rand(len(self.map_shape)) * self.map_shape).astype(np.int32)

    def find_closest(self, target, n=1, /, remove_endpoints=False, src=None):
        remove_mod = remove_endpoints
        if not src:
            src = range(self.graph.node_num)
        remove_target = np.array(self.graph.blocked_nodes).astype(np.int32)
        target = target.astype(np.float32)
        out_ind = np.ones(n).astype(np.int32) * -1
        out_ind[0] = src[0]
        out_dist = np.ones(n).astype(np.float32) * -1
        out_dist[0] = np.linalg.norm(self.graph.nodes[src[0]] - target)
        if len(src) < 2:
            return [out_dist[:n], out_ind[:n]]

        fast_closest(target, self.graph.nodes[src], out_ind, out_dist, n, remove_target, remove_mod)
        if n > 1:
            out_ind = np.array(src)[out_ind]
        return [out_dist[:n], out_ind[:n]]

    def new_from_rand(self, rand_point, closest_node_pos):
        new_node_pos, dist, reached = self.check_obstacle(closest_node_pos, rand_point)
        dist, closest = self.find_closest(new_node_pos, self.overpopulation_num)
        if dist[-1] > self.overpopulation_radius:
            return new_node_pos
        else:
            return np.array([False])

    def add_new_node(self, pos, parent_ind):
        parent = self.graph[parent_ind]
        if pos.any():
            new_node = self.graph.add_node(parent, pos)
            self.opened_nodes.append(new_node.index)
            return new_node

    def check_neighbours(self, node, neighbours):
        for neighbour_ind in neighbours:
            neighbour = self.graph[neighbour_ind]
            _, dist, reached = self.check_obstacle(node.pos, neighbour.pos)
            if reached:
                if dist + node.dist_to_origin < neighbour.dist_to_origin:
                    if neighbour.rank not in [ORIGIN_BLOCKED, ORIGIN]:
                        neighbour.new_parent(node)
                        if neighbour.rank in [ENDPOINT_BLOCKED, ENDPOINT]:
                            neighbour.rank = ENDPOINT
                            self.dist_reached = True
                            self.end_node = neighbour.index

    def step(self):
        # generate random point
        rand_point = self.generate_rand_point()

        # find the closest existing node to generated point
        src = None
        if self.open_closed_lists:
            if np.random.random() > self.open_list_bias:
                src = self.opened_nodes
        _, closest_node_ind = self.find_closest(rand_point, src=src, remove_endpoints=True)
        # skip to the next iteration if none has been found
        if not closest_node_ind.any() or closest_node_ind[0] < 0:
            return False

        closest_node_pos = self.graph.nodes[closest_node_ind[0]]

        # find the best possible new node for given random and closest
        new_node_pos = self.new_from_rand(rand_point, closest_node_pos)
        # skip to the next iteration if none has been found
        if not new_node_pos.any():
            if closest_node_ind not in self.closed_nodes:
                self.closed_nodes.append(*closest_node_ind)
                self.opened_nodes.remove(closest_node_ind)
            return False

        # add new node to graph
        new_node = self.add_new_node(new_node_pos, closest_node_ind[0])

        # check all nodes in the area and rebalance if needed
        if self.star:
            _, neighbours = self.find_closest(rand_point)
            self.check_neighbours(new_node, neighbours)
        return True

    def run(self):
        while self.main_thr.is_alive():
            self.step_lock.acquire()
            self.step()
            self.step_lock.release()
            time.sleep(0.0001)

    def start_thread(self):
        self.tree_thr = thr.Thread(target=self.run)
        self.tree_thr.start()

    def get_path(self):
        node = self.graph[self.end_node]
        self.path = []
        self.path_ind = []
        while node.parent:
            self.path.append(node.pos)
            self.path_ind.append(node.index)
            node = self.graph[node.parent]
        if not self.graph_printed:
            self.graph_printed = True
