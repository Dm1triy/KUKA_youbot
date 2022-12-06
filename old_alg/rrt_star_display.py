import random
import math
import numpy as np

import pygame
import matplotlib.pyplot as plt
from matplotlib import colors
## 1. RRT - random nodes connected to nearest
## 2. RRT* - rrt with rewiring to get path smother

##          Planning
## 1. get random node
## 2. find the nearest created node to the random
## 4. check obstacles between the created node and the random node at a distance of search radius
## 5. create new node between the created node and the random at a distance of search radius or smaller
## 6. connect the new node to one of the neighborhood that will provide the shortest path from the start
## 7. rewire nodes from the new node's neighborhood.
## 8. connect to the goal node.
## 9. calculate final path

class Rrt_star:
    def __init__(self, map, search_radius, goal_radius, neighborhood_radius, total_nodes, robot_radius=0.3):
        self.map = map
        self.robot_radius = robot_radius    # [m]
        self.cell_size = 1/self.map.discrete    # [m]
        # neighborhood_radius should be larger then search_radius
        self.search_radius = search_radius
        self.neighborhood_radius = neighborhood_radius

        #discretes
        self.x_width = self.map.map_size
        self.y_width = self.map.map_size

        self.obstacle_map = np.where(self.map.map_arr == -1, 1, self.map.map_arr)
        self.expand_obstacle_map()

        self.goal_radius = goal_radius
        self.total_nodes = total_nodes

        self.node_dict = dict()

        self.display = self.Display((self.x_width, self.y_width))
        self.display.print_obstacle(self.obstacle_map)

    class Display:
        def __init__(self, winsize):
            pygame.init()
            self.running = True
            self.screen = pygame.display.set_mode(winsize)
            pygame.display.set_caption('RRTstar')
            self.white = 255, 240, 200
            self.black = 20, 20, 40

        def run(self):
            while self.running:
                pygame.display.update()
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        self.running = False

        def print_obstacle(self, obstacle_map):
            obstacle = np.ndarray(shape=(obstacle_map.shape[0], obstacle_map.shape[1], 3))
            obstacle[:] = [255, 240, 200]
            iy, ix = np.where(obstacle_map == 1)
            for i, j in zip(ix, iy):
                for k in range(3):
                    if k == 0:
                        obstacle[i][j][k] = 10
                    if k == 1:
                        obstacle[i][j][k] = 100
                    if k == 2:
                        obstacle[i][j][k] = 10
            self.screen.blit(pygame.surfarray.make_surface(obstacle), (0, 0))

        def display_connection(self, point1, point2):
            black = 20, 20, 40
            pygame.draw.line(self.screen, black, point1, point2)
            pygame.display.update()

        def rewire(self, old, cur, new):
            white = 255, 240, 200
            black = 20, 20, 40
            pygame.draw.line(self.screen, white, old, cur)
            pygame.draw.line(self.screen, black, new, cur)
            pygame.display.update()

        def display_radius(self, point, radius):
            red = 255, 0, 0
            pygame.draw.circle(self.screen, red, (point[0], point[1]), radius)
            pygame.display.update()

        def display_path(self, point1, point2):
            blue = 30, 144, 255
            pygame.draw.line(self.screen, blue, point1, point2, 3)

    class Node:
        def __init__(self, x, y, cost=0, parent=None):
            self.x = x      # indexes
            self.y = y
            self.cost = cost
            self.parent = parent

    def planning(self, sx, sy, gx, gy):
        ix, iy = self.pos_to_cell(sx, sy)
        start_node = self.Node(ix, iy)
        ix, iy = self.pos_to_cell(gx, gy)
        goal_node = self.Node(ix, iy)
        self.node_dict[self.node_index(start_node)] = start_node
        goal_neighbors = dict()
        self.display.display_radius([goal_node.x, goal_node.y], self.goal_radius)
        for i in range(self.total_nodes):
            rand_node = self.Node(random.randrange(self.x_width), random.randrange(self.y_width))
            nearest_node = self.find_nearest_node(rand_node)
            interpolated_node = self.calc_interpolated_node(nearest_node, rand_node)

            if not interpolated_node:
                continue

            interpolated_node.parent = self.node_index(nearest_node)    # temporary parent
            neighbors = self.find_neighbors(interpolated_node)
            parent = self.choose_parent(neighbors, interpolated_node)

            interpolated_node.parent = self.node_index(parent)
            interpolated_node.cost = self.distance(parent, interpolated_node) + parent.cost
            self.node_dict[self.node_index(interpolated_node)] = interpolated_node

            self.display.display_connection([parent.x, parent.y], [interpolated_node.x, interpolated_node.y])

            self.rewire(neighbors, interpolated_node)

            # refactor in the future (find_neighbors -> find_goal_neighbors)
            if self.distance(interpolated_node, goal_node) < self.goal_radius:
                if not self.any_collisions_between(interpolated_node, goal_node):
                    goal_neighbors[self.node_index(interpolated_node)] = interpolated_node
        if len(goal_neighbors) == 0:
            print('Path not found')
            return None, None
        goal_parent = self.best_goal_neighbor(goal_neighbors, goal_node)
        goal_node.parent = self.node_index(goal_parent)
        self.node_dict[self.node_index(goal_node)] = goal_node

        x, y = self.calc_final_path(goal_node)
        return x, y

    def calc_final_path(self, goal_node):
        x, y = self.cell_to_pos(goal_node.x, goal_node.y)
        path_x = np.array([x])
        path_y = np.array([y])
        iter_node = goal_node
        while iter_node.parent:
            iter_node = self.node_dict[iter_node.parent]
            #display
            temp = self.pos_to_cell(path_x[-1], path_y[-1])
            self.display.display_path((iter_node.x, iter_node.y), (temp[0], temp[1]))
            #/////////////////////
            x, y = self.cell_to_pos(iter_node.x, iter_node.y)
            path_x = np.append(path_x, x)
            path_y = np.append(path_y, y)
        pygame.display.update()
        return np.flip(path_x), np.flip(path_y)

    def rewire(self, neighbors_dict, new_node):
        for i in neighbors_dict:
            node = neighbors_dict[i]
            d = self.distance(node, new_node)
            if new_node.cost + d < node.cost and i != new_node.parent \
                    and not self.any_collisions_between(new_node, node):
                old_parent = self.node_dict[node.parent]     # for display
                node.parent = self.node_index(new_node)
                node.cost = new_node.cost + d
                self.display.rewire([old_parent.x, old_parent.y], [node.x, node.y],
                                    [new_node.x, new_node.y])

    def choose_parent(self, neighbors_dict, new_node):
        best_parent = neighbors_dict[new_node.parent]
        for n in neighbors_dict.values():
            if n.cost + self.distance(n, new_node) < best_parent.cost + self.distance(best_parent, new_node) \
                    and not self.any_collisions_between(n, new_node):
                best_parent = n
        return best_parent

    def find_neighbors(self, new_node):
        neighbors = dict()
        for i in self.node_dict:
            if self.distance(self.node_dict[i], new_node) < self.neighborhood_radius:
                neighbors[i] = self.node_dict[i]
        return neighbors

    def calc_interpolated_node(self, real, imagine):
        d = self.distance(real, imagine)
        if d < self.search_radius and not self.any_collisions_between(real, imagine):
            return imagine
        else:
            ## checking all cells for an obstacle at a distance less than search_radius
            for r in range(1, self.search_radius):
                # (imagine.x - real.x)/d - unit direction vector
                # 0.5 should be removed after tests
                x = int(real.x + (imagine.x - real.x) * r / d)
                y = int(real.y + (imagine.y - real.y) * r / d)
                if self.obstacle_map[y][x] == 1 and r == 1:
                    return None
                elif self.obstacle_map[y][x] == 1:
                    x = int(real.x + (imagine.x - real.x) * (r-1) / d)
                    y = int(real.y + (imagine.y - real.y) * (r-1) / d)
                    return self.Node(x, y)
            return self.Node(int(real.x + (imagine.x - real.x) * self.search_radius / d),
                             int(real.y + (imagine.y - real.y) * self.search_radius / d))

    def find_nearest_node(self, target_node):
        nearest_node = list(self.node_dict.values())[0]
        for n in self.node_dict.values():
            if self.distance(n, target_node) < self.distance(nearest_node, target_node):
                nearest_node = n
        return nearest_node

    def node_index(self, node):
        return node.y * self.x_width + node.x

    def cell_to_pos(self, x, y):
        # mb (x - 1 - width) * cell_size
         return np.round((x - self.x_width//2) * self.cell_size, 2), \
                np.round((self.y_width//2 - y) * self.cell_size, 2)  # top y_position is positive but y_indx < height

    def pos_to_cell(self, x, y):
       # top y_position is positive but y_indx < height
       return int(self.x_width//2 + x/self.cell_size), int(self.y_width//2 - y/self.cell_size)

    @staticmethod
    def distance(node1, node2):
        return math.hypot(node1.x - node2.x, node1.y - node2.y)

    def best_goal_neighbor(self, goal_neighbors, goal_node):
        best_node = list(goal_neighbors.values())[0]
        for n in goal_neighbors.values():
            if n.cost + self.distance(n, goal_node) < \
                    best_node.cost + self.distance(best_node, goal_node):
                best_node = n
        return best_node

    # expand with squares not circles
    def expand_obstacle_map(self):
        i_oy, i_ox = np.where((self.obstacle_map == 1))
        cell_radius = int(np.round(self.robot_radius / self.cell_size))
        for ox, oy in zip(i_ox, i_oy):
            for j in range(oy-cell_radius, oy+cell_radius):
                for i in range(ox-cell_radius, ox+cell_radius):
                    if 0 <= i < self.x_width and 0 <= j < self.y_width:
                        self.obstacle_map[j][i] = 1

    def any_collisions_between(self, node1, node2):
        d = self.distance(node1, node2)
        for r in range(int(d)):
            # 0.5 should be removed after tests
            x = int(node1.x + (node2.x - node1.x) * r / d)
            y = int(node1.y + (node2.y - node1.y) * r / d)
            if self.obstacle_map[y][x] == 1:
                return True
        return False

if __name__ == '__main__':
    class Map_test:
        def __init__(self):
            self.discrete = 20
            self.map_size = 350
            self.map_arr = np.array([[0] * self.map_size] * self.map_size)
            # y numbering from top
            #             x    y
            self.start = 200, 150
            self.goal = 120, 20

        def cell_to_pos(self, x, y):
            return (x - self.map_size//2)/self.discrete, (self.map_size//2 - y)/self.discrete

        def pos_to_cell(self, x, y):
            return int(self.map_size//2 + x*self.discrete), int(self.map_size//2 - y*self.discrete)

        def add_obstacle(self):
            self.map_arr[50][50:] = 1  # horizontal obstacle
            for i in range(30, 70):
                self.map_arr[i][50:] = 1
            for i in range(150, self.map_size):  # vertical
                self.map_arr[i][70:110] = 1
            for i in range(50, self.map_size - 80):
                self.map_arr[i][270:300] = 1

        def print_map(self):
            printed = self.map_arr.copy()
            #                                   free, obstacle, start, goal, interpolated
            map_colors = colors.ListedColormap(['white', 'yellow', 'green', 'red'])
            # if points aren't visible due to the scaling
            for i in range(self.start[1] - 3, self.start[1] + 3):
                for j in range(self.start[0] - 3, self.start[0] + 3):
                    printed[i][j] = 2  # start
            for i in range(self.goal[1] - 3, self.goal[1] + 3):
                for j in range(self.start[0] - 3, self.start[0] + 3):
                    printed[i][j] = 3  # goal

            plt.figure(figsize=(8, 8))
            plt.pcolor(printed[::-1], cmap=map_colors, edgecolors='k', linewidths=0.03)
            plt.show()


    new_map = Map_test()
    new_map.add_obstacle()
    start_cell = new_map.start
    goal_cell = new_map.goal
    start_pos = new_map.cell_to_pos(start_cell[0], start_cell[1])   # [m]
    goal_pos = new_map.cell_to_pos(goal_cell[0], goal_cell[1])  # [m]

    alg = Rrt_star(new_map, 20, 20, 40, 1500)
    path = alg.planning(start_pos[0], start_pos[1], goal_pos[0], goal_pos[1])
    print(path[0], path[1])
    alg.display.run()

