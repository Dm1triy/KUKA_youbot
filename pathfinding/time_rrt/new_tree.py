import random
import math

from pathfinding.time_rrt.fast_math import *
from pathfinding.time_rrt.rrt_graph import *


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
        self.min_speed = 0
        self.max_speed = 3
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

    def check_obstacle_old(self, point1, point2):
        shift_vector = (point2.astype(np.float32) - point1.astype(np.float32)).astype(np.float32)
        transition = np.linalg.norm(shift_vector[:self.time_dimension])
        time = shift_vector[self.time_dimension]
        if time <= 0:
            return np.array(False), None, False
        if not self.min_speed <= transition/time <= self.max_speed:
            return np.array(False), None, False
        iters = int(sum(abs(shift_vector)) * 2)
        if iters == 0:
            return np.array(False), None, False
        shift_vector = shift_vector / iters
        all_shift = shift_vector
        c_point = np.array(False)
        iters_made = 0
        for i in range(1, iters + 1):
            iters_made = i
            all_shift = np.copy(shift_vector * i)
            c_point = np.around(point1 + shift_vector * i).astype(np.int64)
            try:
                if self.bool_map[tuple(c_point)]:
                    i -= 1
                    iters_made = i
                    break
                if abs(np.linalg.norm(shift_vector * i)) >= self.growth_factor:
                    break
            except IndexError:
                break
        if np.linalg.norm(all_shift) < self.e or not c_point.any():
            return np.array(False), None, False
        if iters_made > 1:
            return c_point, np.linalg.norm(all_shift), iters_made == iters
        else:
            return np.array(False), None, False

    def check_obstacle(self, point1, point2):
        info = np.zeros(len(point1)+2).astype(np.float32)
        check_obstacle(point1, point2, self.bool_map, self.growth_factor, self.e, self.min_speed, self.max_speed, info)
        node, dist, reached = info[:3], info[3], info[4]
        node = node.astype(np.int32)
        reached = bool(reached)
        return node, dist, reached

    def step(self):
        self.add_random()

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
        _, closest_node_ind = find_closest(new_node_pos, self.graph.nodes, remove_target=self.graph.blocked_nodes)
        alph, th = np.random.random(2)
        alph = alph*2*math.pi
        th = self.min_speed_ang + th * (self.max_speed_ang-self.min_speed_ang)
        z = self.growth_factor*math.sin(th)
        lo = self.growth_factor*math.cos(th)
        x = lo*math.sin(alph)
        y = lo*math.cos(alph)
        closest_node = self.graph.nodes[closest_node_ind[0]]
        new_node_pos = closest_node + np.array([x, y, z]).astype(int)
        found_node_pos, dist, reached = self.check_obstacle(self.graph.nodes[closest_node_ind[0]], new_node_pos)
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
