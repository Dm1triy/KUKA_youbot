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

    def fast_math_check_obstacle(self, point1, point2, info):
        check_obstacle(point1, point2, self.bool_map, self.growth_factor, self.e, self.min_speed, self.max_speed, info)

    def fast_math_closest(self, target, src, out_ind, out_dist, n, inv):
        fast_closest(target, src, out_ind, out_dist, n, self.max_speed, self.min_speed, inv)


