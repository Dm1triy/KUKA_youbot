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

    def check_obstacle(self, point1, point2):
        shift_vector = (point2.astype(np.float32) - point1.astype(np.float32)).astype(np.float32)
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

    def step(self):
        self.add_random()

    def find_best_connection(self, new_node_pos, neighbours):
        neighbours = [self.graph[i] for i in neighbours]
        neighbours.sort(key=lambda x: (x.rank, x.dist_to_origin), reverse=False)
        have_parent = False
        for neighbour in neighbours:
            if neighbour.rank == ORIGIN_BLOCKED:
                continue
            _, dist, reached = self.check_obstacle(new_node_pos, neighbour.pos)
            if reached:
                if not have_parent:
                    if neighbour.rank != ENDPOINT_BLOCKED:
                        new_node = self.graph.add_node(neighbour, new_node_pos)
                        have_parent = True
                else:
                    if dist + new_node.dist_to_origin < neighbour.dist_to_origin:
                        if neighbour.rank not in [ORIGIN_BLOCKED, ORIGIN]:
                            neighbour.new_parent(new_node)
                            if neighbour.rank == ENDPOINT_BLOCKED:
                                neighbour.rank = ENDPOINT
                                self.graph.target_ind.remove(neighbour.index)
                                self.dist_reached = True
                                self.end_node = neighbour.index

    def add_node_to_closest(self, new_node_pos):
        _, closest_node = find_closest(new_node_pos, self.graph.nodes, remove_target=self.graph.target_ind)
        new_node_pos, dist, reached = self.check_obstacle(self.graph.nodes[closest_node[0]], new_node_pos)
        if new_node_pos.any():
            _, neighbors = find_closest(new_node_pos, self.graph.nodes, 10, dist_limit=self.growth_factor)
            self.find_best_connection(new_node_pos, neighbors)
        return reached

    def add_random(self):
        random_point = (np.random.rand(len(self.map_shape)) * self.map_shape).astype(np.int32)
        self.random_point = random_point
        self.add_node_to_closest(random_point)

    def get_path(self):
        node = self.graph[self.end_node]
        self.path = []
        self.path_ind = []
        while node.parent:
            node = self.graph[node.parent]
            self.path.append(node.pos)
            self.path_ind.append(node.index)
        if not self.graph_printed:
            self.graph_printed = True
