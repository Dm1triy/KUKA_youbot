import time

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors
from matplotlib import collections as mc
from path_planning.adaptAstar import Astar


class PathPlanner:
    def __init__(self, robot):
        self.robot = robot

        self.cell_size = 0.05
        self.map_width = 401
        self.map_height = 401
        self.start_cell = self.map_width//2, self.map_height//2
        self.map = np.array([[0] * self.map_width] * self.map_height)

    def run_Astar(self, target, weights=None, surface=False):
        pos, _ = self.robot.lidar
        pos = self.pos2cell(pos)
        target = self.pos2cell(target)

        fig, ax = self.init_plot((200, 200), pos)
        pos_circle = plt.Circle(pos, 1, color='b')
        target_circle = plt.Circle(target, 1, color='r')
        c1 = ax.add_patch(pos_circle)
        ax.add_patch(target_circle)

        if surface:
            index = np.where(weights == 1.6)
            ax.scatter(index[1], index[0], color='r', s=3)

        alg = Astar(np.array(pos), np.array(target), self.map, adapt_weights=weights, adapt_flag=surface)
        i = 0
        while not alg.dist_reached:
            alg.step()
            if i % 200 == 0:
                node_indx = self.reshape_array(alg.nodes)
                ax.scatter(node_indx[0]+0.5, node_indx[1]+0.5, color='y', s=2, marker='H', linewidths=1)
                plt.pause(0.1)
            i += 1
        path = alg.get_path()
        if not path:
            return
        path = self.reshape_array(path)
        ax.plot(path[0] + 0.5, path[1] + 0.5, color='r')
        c1.remove()
        self.move_along_path(path, ax)
        plt.show()

    def init_plot(self, shape, pos):
        lower_x = pos[0] - shape[0]//2
        upper_x = pos[0] + shape[0]//2
        lower_y = pos[1] - shape[1]//2
        upper_y = pos[1] + shape[1]//2

        fig, ax = plt.subplots(figsize=(15, 8))
        cmap = colors.ListedColormap(['White', 'Green'])
        ax.pcolor(self.map, cmap=cmap, snap=True)
        ax.set_xlim(lower_x, upper_x)
        ax.set_ylim(lower_y, upper_y)
        ax.invert_yaxis()

        majorx_ticks = np.arange(lower_x, upper_x - 1, 10)
        minorx_ticks = np.arange(lower_x, upper_x - 1, 1)
        majory_ticks = np.arange(lower_y, upper_y - 1, 10)
        minory_ticks = np.arange(lower_y, upper_y - 1, 1)

        ax.set_xticks(majorx_ticks)
        ax.set_xticks(minorx_ticks, minor=True)
        ax.set_yticks(majory_ticks)
        ax.set_yticks(minory_ticks, minor=True)
        ax.grid(which='major', linewidth=1.2)
        ax.grid(which='minor')
        fig.tight_layout()
        return fig, ax

    def move_along_path(self, path, ax):
        c2 = ax.add_patch(plt.Circle((path[0][-1], path[1][-1]), 1, color='b'))
        for i in range(2, len(path[0]), 5):
            next_pos = path[0][-i], path[1][-i]
            next_pos = self.cell2pos(next_pos)
            self.robot.go_to(next_pos[0], next_pos[1], initial_speed=1)
            while self.robot.going_to_target_pos:
                time.sleep(0.1)
            cur_pos, _ = self.robot.lidar
            cur_pos = self.pos2cell(cur_pos)
            c2.remove()
            c2 = ax.add_patch(plt.Circle(cur_pos, 1, color='b'))
            plt.pause(0.1)

    def pos2cell(self, pos):
        return int(self.start_cell[0] + pos[0] / self.cell_size), \
               int(self.start_cell[1] - pos[1] / self.cell_size)

    def cell2pos(self, cell):
        return np.round((cell[0] - self.start_cell[0]) * self.cell_size, 2), \
               np.round((self.start_cell[1] - cell[1]) * self.cell_size, 2)

    @staticmethod
    def reshape_array(array):
        """
        Convert [[i1, j1], [i2, j2], ...] into [[i1, i2, ...], [j1, j1, ...]]
        """
        array = np.array(array)
        array = np.array(np.split(array, 2, axis=1))
        array = array.reshape(array.shape[0], array.shape[1])
        return array


if __name__ == "__main__":

    class Robot:
        def __init__(self):
            # self.pos = np.random.uniform(-3, 3, 2)
            self.pos = 0, 0
            self.going_to_target_pos = False

        def go_to(self, x, y):
            time.sleep(0.1)
            self.pos = x, y

        @property
        def lidar(self):
            return self.pos, None

    robot = Robot()
    pp = PathPlanner(robot)
    goal = 0, 3
    # weights = np.random.uniform(1, 10, (401, 401))
    weights = np.ones((401, 401))
    weights[161:181, 191:211] = 100
    pp.run_Astar(goal, weights, True)
