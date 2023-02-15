import random
import math
import time

from pathfinding.time_rrt.fast_math import *
from pathfinding.time_rrt.rrt_graph import *

import threading as thr


class Tree:
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
        self.star = True
        # speed in space discretes per time discretes
        self.min_speed = 2
        self.max_speed = 5
        self.min_speed_ang = math.atan(self.min_speed)
        self.max_speed_ang = math.atan(self.max_speed)
        self.time_dimension = 2

        self.graph = Graph(start_point=start_point, start_point_available=start_point_available, end_point=end_point)

        map_shape = self.bool_map.shape
        self.map_shape = (map_shape - np.ones(len(map_shape))).astype(np.int32)
        self.stuck = 0
        self.force_random = 0
        self.dist_reached = False
        self.end_node = -1
        self.path = []
        self.path_ind = []
        self.graph_printed = False
        self.main_thr = thr.main_thread()

        self.growth_factor = growth_factor
        self.e = e
        self.end_dist = end_dist
        self.rrt_star_rad = rrt_star_rad

        if not start_point.any():
            print("No start point")
        assert start_point.any()
        # if not end_point.any():
        #    print("No end point")
        assert end_point.any()
        if not bin_map.any():
            print("No bin_map")
        assert bin_map.any()

        self.path_lock = thr.Lock()
        self.tree_thr = thr.Thread(target=self.run)
        self.tree_thr.start()

    def check_obstacle(self, point1, point2):
        info = np.zeros(len(point1)+2).astype(np.float64)
        check_obstacle(point1, point2, self.bool_map, self.growth_factor, self.e, self.min_speed, self.max_speed, info)
        node, dist, reached = info[:3], info[3], info[4]
        node = node.astype(np.float64)
        reached = bool(reached)
        return node, dist, reached

    def step(self):
        self.path_lock.acquire()
        self.add_random()
        self.path_lock.release()

    def run(self):
        while self.main_thr.is_alive():
            self.step()
            time.sleep(0.006)

    def find_best_connection(self, new_node_pos, neighbours):
        neighbours = [self.graph[i] for i in neighbours]
        neighbours.sort(key=lambda x: (x.rank, x.dist_to_origin), reverse=False)
        have_parent = False
        for neighbour in neighbours:
            if neighbour.rank == ORIGIN_BLOCKED:
                continue
            if not have_parent:
                _, dist, reached = self.check_obstacle(neighbour.pos, new_node_pos)
            else:
                _, dist, reached = self.check_obstacle(new_node_pos, neighbour.pos)
            if reached:
                if not have_parent:
                    if neighbour.rank not in [ENDPOINT_BLOCKED, ENDPOINT, ORIGIN_BLOCKED]:
                        new_node = self.graph.add_node(neighbour, new_node_pos)
                        have_parent = True
                else:
                    if dist + new_node.dist_to_origin < neighbour.dist_to_origin:
                        if neighbour.rank not in [ORIGIN_BLOCKED, ORIGIN]:
                            neighbour.new_parent(new_node)
                            if neighbour.rank == ENDPOINT_BLOCKED:
                                neighbour.rank = ENDPOINT
                                self.graph.update_endpoints(neighbour)
                                self.graph.unblock_end(neighbour.index)
                                self.dist_reached = True
                                self.end_node = neighbour.index

    def add_node_to_closest(self, new_node_pos):
        _, closest_node = find_closest(new_node_pos, self.graph.nodes, 10, remove_target=self.graph.blocked_nodes)
        for i in range(10):
            found_node_pos, dist, reached = self.check_obstacle(self.graph.nodes[closest_node[i]], new_node_pos)
            if dist:
                break
        _, neighbors = find_closest(found_node_pos, self.graph.nodes, 10, dist_limit=self.growth_factor)
        self.find_best_connection(found_node_pos, neighbors)


    def add_node_to_closest_with_speed(self, new_node_pos):
        dist, closest_node_ind = find_closest(new_node_pos, self.graph.nodes, remove_target=self.graph.blocked_nodes)
        closest_node_pos = self.graph.nodes[closest_node_ind[0]]
        dist_to_closest = np.linalg.norm(closest_node_pos[:2])
        delta_pos = new_node_pos - closest_node_pos
        time_delta = delta_pos[2]
        curr_th = math.atan2(dist_to_closest, time_delta)
        th = max(self.min_speed_ang, min(self.max_speed_ang, curr_th))
        z = dist_to_closest*math.cos(th)
        x = math.sin(th)*delta_pos[0]
        y = math.sin(th)*delta_pos[1]

        new_node_pos = closest_node_pos + np.array([x, y, z]).astype(int)
        found_node_pos, dist, reached = self.check_obstacle(closest_node_pos, new_node_pos)
        if found_node_pos.any():
            _, neighbors = find_closest(found_node_pos, self.graph.nodes, 10, dist_limit=self.growth_factor)
            self.find_best_connection(found_node_pos, neighbors)

    def add_random(self):
        random_point = (np.random.rand(len(self.map_shape)) * self.map_shape).astype(np.int32)
        self.random_point = random_point
        self.add_node_to_closest_with_speed(random_point)
        #self.add_node_to_closest(random_point)

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
