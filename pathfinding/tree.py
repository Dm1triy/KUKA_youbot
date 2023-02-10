import numpy as np
import scipy
import scipy.spatial
from numba import njit


@njit(fastmath=True)
def fast_closest(target, src, out_ind, out_dist, n, remove_target):

    for p in range(1, len(src)):
        is_target = False
        for rm_t in remove_target:
            if p == rm_t:
                is_target = True
                break
        if is_target:
            continue
        point = src[p]
        dist = np.linalg.norm(point - target)
        for i in range(n):
            if out_dist[i] > dist:
                out_dist[i] = dist
                out_ind[i] = p
                for j in range(n - 1, i, -1):
                    out_ind[j] = out_ind[j - 1]
                    out_dist[j] = out_dist[j - 1]
                break


def find_closest(target, src, n=1, /, dist_limit=None, remove_target=(-1, -1)):
    remove_target = np.array(remove_target).astype(np.int32)
    target = target.astype(np.float32)
    out_ind = np.zeros(n).astype(np.int32)
    out_dist = np.zeros(n).astype(np.float32)
    out_dist[0] = np.linalg.norm(src[0] - target)
    fast_closest(target, src, out_ind, out_dist, n, remove_target)
    return [out_dist[:n], out_ind[:n]]


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
        if not start_point_available:
            start_point_available = [True]*len(start_point)

        # start points
        self.graph = dict()
        self.graph[0] = [None, [], 0]
        self.nodes = np.array([start_point[0]]).astype(np.float32)
        self.node_num = 1
        for i in range(1, len(start_point_available)):
            if start_point_available[i]:
                self.graph[i] = [i - 1, [], 0]
            else:
                self.graph[i] = [i - 1, [], -1]
            self.graph[i - 1][1].append(i)
            self.nodes = np.append(self.nodes, [start_point[i]], axis=0).astype(np.int32)
            self.node_num += 1


        # end points
        self.target_ind = []
        self.graph[self.node_num] = [None, [], float("inf")]
        self.nodes = np.append(self.nodes, [end_point[0]], axis=0).astype(np.int32)
        self.target_ind.append(self.node_num)
        self.node_num += 1
        for i in range(0, len(end_point)):
            nn = self.node_num
            self.graph[nn] = [nn-1, [], float("inf")]
            self.graph[nn-1][1].append(nn)
            self.nodes = np.append(self.nodes, [end_point[i]], axis=0).astype(np.int32)
            self.target_ind.append(self.node_num)
            self.node_num += 1

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
        iters = int(sum(abs(shift_vector))*2)
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
                if np.linalg.norm(shift_vector * i) >= self.growth_factor:
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
        neighbours.sort(key=lambda x: x[-1], reverse=False)
        have_parent = False
        for i in neighbours:
            neighbour_id, neighbour_node, neighbour_parent, neighbour_children, neighbour_dist_to_orig = i
            if neighbour_dist_to_orig == -1:
                continue
            _, dist, reached = self.check_obstacle(new_node, neighbour_node)
            if reached:
                if not have_parent:
                    self.nodes = np.append(self.nodes, [new_node], axis=0).astype(np.int32)
                    self.graph[self.node_num] = [neighbour_id, [], dist + neighbour_dist_to_orig]
                    self.graph[neighbour_id][1].append(self.node_num)
                    self.node_num += 1
                    have_parent = True
                else:
                    if dist + self.graph[self.node_num - 1][2] < neighbour_dist_to_orig:
                        if neighbour_id in self.graph[neighbour_parent][1]:
                            self.graph[neighbour_id][0] = self.node_num - 1
                            self.graph[neighbour_parent][1].remove(neighbour_id)
                            self.graph[self.node_num - 1][1].append(neighbour_id)
                            self.rebalance(neighbour_id, self.graph[neighbour_id][2] - dist - self.graph[self.node_num - 1][2])
                            if neighbour_id in self.target_ind:

                                self.target_ind.remove(neighbour_id)
                                self.dist_reached = True
                                self.end_node = self.node_num-1

    def rebalance(self, node_num, delta):
        self.graph[node_num][2] -= delta
        for i in self.graph[node_num][1]:
            self.rebalance(i, delta)

    def add_node_to_closest(self, new_node):
        _, closest_node = find_closest(new_node, self.nodes, remove_target=self.target_ind)
        node, dist, reached = self.check_obstacle(self.nodes[closest_node[0]], new_node)
        if node.any():
            _, neighbors = find_closest(node, self.nodes, 10, dist_limit=self.growth_factor)
            self.find_best_connection(node, neighbors)

        return reached

    def add_random(self):
        random_point = (np.random.rand(len(self.map_shape)) * self.map_shape).astype(np.int32)
        self.random_point = random_point
        self.add_node_to_closest(random_point)

    def add_near_end(self):
        reached = self.add_node_to_closest(self.end_point)
        if reached:
            print("done")
            self.dist_reached = True
            self.end_node = self.node_num - 1
        else:
            self.stuck += 1

    def get_path(self):
        node_num = self.end_node
        self.path = []
        self.path_ind = []
        while node_num:
            self.path.append(self.nodes[node_num])
            self.path_ind.append(node_num)
            node_num = self.graph[node_num][0]
        self.path.append(self.nodes[node_num])
        self.path_ind.append(node_num)
        if not self.graph_printed:
            self.graph_printed = True