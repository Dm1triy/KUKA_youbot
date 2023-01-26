import numpy as np
import scipy
import scipy.spatial


def find_closest(target, src=np.array(False), n=1, /, dist_limit=None):
    out = [[], []]
    if src.any():
        nodes_tree = scipy.spatial.cKDTree(src)
        _res = nodes_tree.query(target, n)
        res = list(_res)
        if isinstance(_res[0], float):
            res[0] = [_res[0]]
        if isinstance(_res[1], int):
            res[1] = [_res[1]]

        for i in range(len(res[1])):
            if res[1][i] < len(src) and (not dist_limit or res[0][i] < dist_limit):
                out[0].append(res[0][i])
                out[1].append(res[1][i])
        if len(out) > 0:
            return out
    return [None, None]


class RRT:
    def __init__(self, /,
                 start_point=None,
                 end_point=np.array(False),
                 bin_map=None,
                 growth_factor=20,
                 e=0.04,
                 end_dist=25,
                 rrt_star_rad=20):
        self.end_point = end_point
        self.bool_map = bin_map

        self.star = True

        self.graph = dict()
        self.graph[0] = [None, [], 0]
        self.nodes = np.array(start_point).astype(np.uint32)
        self.node_num = 1
        map_shape = self.bool_map.shape
        self.map_shape = (map_shape - np.ones(len(map_shape))).astype(np.uint32)
        self.stuck = 0
        self.force_random = 0
        self.dist_reached = False
        self.end_node = -1
        self.path = []
        self.graph_printed = False

        self.growth_factor = growth_factor
        self.e = e
        self.end_dist = end_dist
        self.rrt_star_rad = rrt_star_rad

        if not start_point.any():
            print("No start point")
        assert start_point.any()
        #if not end_point.any():
        #    print("No end point")
        assert end_point.any()
        if not bin_map.any():
            print("No bin_map")
        assert bin_map.any()

    def check_obstacle(self, point1, point2):
        shift_vector = (point2 - point1).astype(np.int32)
        iters = sum(abs(shift_vector))
        shift_vector = shift_vector / iters
        all_shift = shift_vector
        c_point = np.array(False)
        iters_made = 0
        for i in range(1, iters + 1):
            iters_made = i
            all_shift = np.copy(shift_vector * i)
            c_point = np.around(point1 + shift_vector * i).astype(np.int64)
            if self.bool_map[tuple(c_point)]:
                i -= 1
                iters_made = i
                break
            if np.linalg.norm(shift_vector * i) >= self.growth_factor:
                break
        if np.linalg.norm(all_shift) < self.e or not c_point.any():
            return np.array(False), None, False
        if iters_made > 1:
            return c_point, np.linalg.norm(all_shift), iters_made == iters
        else:
            return np.array(False), None, False

    def step(self):
        if self.end_point.any():
            node = self.nodes[self.node_num - 1]
            dist = np.linalg.norm(node - self.end_point)
            if self.dist_reached:
                self.add_random()
            if self.force_random:
                self.add_random()
                self.force_random -= 1
            elif dist < self.end_dist and self.force_random == 0:
                self.add_near_end()
                if self.stuck > 10:
                    self.stuck = 0
                    self.force_random = 50
            else:
                self.add_random()
        else:
            self.add_random()

    def find_best_connection(self, new_node, neighbours):
        neighbours = [[i, self.nodes[i], *self.graph[i]] for i in neighbours]
        neighbours.sort(key=lambda x: x[-1])
        have_parent = False
        for i in neighbours:
            _, dist, reached = self.check_obstacle(new_node, i[1])
            if reached:
                if not have_parent:
                    self.nodes = np.append(self.nodes, [new_node], axis=0).astype(np.uint32)
                    self.graph[self.node_num] = [i[0], [], dist + self.graph[i[0]][2]]
                    self.graph[i[0]][1].append(self.node_num)
                    self.node_num += 1
                    have_parent = True
                else:
                    if dist + self.graph[self.node_num - 1][2] < self.graph[i[0]][2]:
                        if i[0] in self.graph[i[2]][1]:
                            self.graph[i[0]][0] = self.node_num - 1
                            self.graph[i[2]][1].remove(i[0])
                            self.graph[self.node_num - 1][1].append(i[0])
                            self.rebalance(i[0], self.graph[i[0]][2] - dist - self.graph[self.node_num - 1][2])

    def rebalance(self, node_num, delta):
        self.graph[node_num][2] -= delta
        for i in self.graph[node_num][1]:
            self.rebalance(i, delta)

    def add_node_to_closest(self, new_node):
        _, closest_node = find_closest(new_node, self.nodes)
        if closest_node:
            node, dist, _ = self.check_obstacle(self.nodes[closest_node[0]], new_node)
            if node.any():
                if self.star:
                    _, neighbors = find_closest(node, self.nodes, 10, dist_limit=self.growth_factor)
                    self.find_best_connection(node, neighbors)
                else:
                    self.nodes = np.append(self.nodes, [node], axis=0).astype(np.uint32)
                    self.graph[self.node_num] = [closest_node[1], [], dist + self.graph[closest_node[1]][2]]
                    self.graph[closest_node[1]][1].append(self.node_num)
                    self.node_num += 1

                return node
            else:
                return np.array(False)

    def add_random(self):
        random_point = (np.random.rand(len(self.map_shape)) * self.map_shape).astype(np.uint32)
        self.random_point = random_point
        self.add_node_to_closest(random_point)

    def add_near_end(self):
        node = self.add_node_to_closest(self.end_point)
        if node.any() and node[0] == self.end_point[0] and node[1] == self.end_point[1]:
            print("done")
            self.dist_reached = True
            self.end_node = self.node_num - 1
        else:
            self.stuck += 1

    def get_path(self):
        node_num = self.end_node
        self.path = []
        while node_num:
            self.path.append(self.nodes[node_num])
            node_num = self.graph[node_num][0]
        self.path.append(self.nodes[node_num])
        if not self.graph_printed:
            self.graph_printed = True
