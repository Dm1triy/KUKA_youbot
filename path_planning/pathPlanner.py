import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors
from matplotlib import collections as mc
import threading as thr
import math
from adaptAstar import Astar


class PathPlanner:
    def __init__(self, robot, surf_map):
        self.robot = robot
        self.surf = surf_map

        self.cell_size = 0.05
        self.map_width = 121
        self.map_height = 121
        self.start_cell = self.map_width//2, self.map_height//2
        self.map = np.array([[0] * self.map_width] * self.map_height)

    def run_Astar(self, target, surface=False):
        fig, ax = self.init_plot()
        pos, _ = self.robot.lidar
        pos = self.pos2cell(pos)
        target = self.pos2cell(target)
        pos_circle = plt.Circle(pos, 1, color='b')
        target_circle = plt.Circle(target, 1, color='r')
        ax.add_patch(pos_circle)
        ax.add_patch(target_circle)

        alg = Astar(np.array(pos), np.array(target), self.map)
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
        plt.show()

    def init_plot(self):
        fig, ax = plt.subplots(figsize=(15, 8))
        cmap = colors.ListedColormap(['White', 'Blue'])
        ax.pcolor(self.map, cmap=cmap, snap=True)
        ax.set_xlim(0, self.map_width)
        ax.set_ylim(0, self.map_height)
        ax.invert_yaxis()

        majorx_ticks = np.arange(0, self.map_width - 1, 10)
        minorx_ticks = np.arange(0, self.map_width - 1, 1)
        majory_ticks = np.arange(0, self.map_height - 1, 10)
        minory_ticks = np.arange(0, self.map_height - 1, 1)

        ax.set_xticks(majorx_ticks)
        ax.set_xticks(minorx_ticks, minor=True)
        ax.set_yticks(majory_ticks)
        ax.set_yticks(minory_ticks, minor=True)
        ax.grid(which='major', linewidth=1.2)
        ax.grid(which='minor')
        fig.tight_layout()
        return fig, ax

    def pos2cell(self, pos):
        return int(self.start_cell[0] + pos[0] / self.cell_size), \
               int(self.start_cell[1] - pos[1] / self.cell_size)

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
            self.pos = -2, 0

        @property
        def lidar(self):
            return self.pos, None

    robot = Robot()
    pp = PathPlanner(robot, None)
    goal = 1, 1
    pp.run_Astar(goal)
