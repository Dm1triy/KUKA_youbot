import random
import math
import time

from pathfinding.time_rrt.fast_math_for_time_rrt import *
from pathfinding.vanilla_rrt.rrt_graph import *
from pathfinding.vanilla_rrt.rrt_tree import TreeRRT

import threading as thr


class TimeTree(TreeRRT):
    def __init__(self, /,
                 **kwargs):
        super().__init__(**kwargs)

        # speed in space discretes per time discretes
        self.min_speed = 5
        self.max_speed = 10
        self.min_speed_ang = math.atan(self.min_speed)
        self.max_speed_ang = math.atan(self.max_speed)
        self.time_dimension = 2

    def check_obstacle(self, point1, point2):
        info = np.zeros(len(point1) + 2).astype(np.float64)
        check_obstacle(point1, point2, self.bool_map, self.growth_factor, self.e, self.min_speed, self.max_speed, info)
        node, dist, reached = info[:3], info[3], info[4]
        node = node.astype(np.float64)
        reached = bool(reached)
        return node, dist, reached

    def find_closest(self, target, n=1, /, src=None, remove_endpoints=False):
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

        fast_closest(target, self.graph.nodes[src], out_ind, out_dist, n, remove_target, remove_mod, self.max_speed,
                     self.min_speed)
        if n > 1:
            out_ind = np.array(src)[out_ind]
        return [out_dist[:n], out_ind[:n]]