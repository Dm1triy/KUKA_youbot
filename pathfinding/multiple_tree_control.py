import threading as thr
import numpy as np
from pathfinding.time_rrt.time_tree import TimeTree
from pathfinding.vanilla_rrt.constants import *


class MultiTree:
    def __init__(self, points, bin_map):
        self.main_thr = thr.main_thread()
        self.trees = []
        self.active_trees_ind = [0]
        self.origins = [[]]
        self.endpoints = [[]]
        origin_availability = [True] * (points[0][2][1] - points[0][2][0])
        for point in range(1, len(points)):
            orig, end = points[point - 1], points[point]
            orig_point = [[orig[0], orig[1], i] for i in range(orig[2][0], orig[2][1])]
            end_point = [[end[0], end[1], i] for i in range(end[2][0], end[2][1])]
            tree = TimeTree(start_point=np.array(orig_point), end_point=np.array(end_point), bin_map=bin_map,
                            start_point_available=origin_availability)
            self.trees.append(tree)
            origin_availability = [False] * len(end_point)

    def run(self):
        while self.main_thr.is_alive():
            for tree_ind in self.active_trees_ind:
                curr_tree = self.trees[tree_ind]
                curr_tree.step()
                if curr_tree.dist_reached:
                    if tree_ind + 1 < len(self.trees):
                        next_tree = self.trees[tree_ind + 1]
                        if tree_ind+1 not in self.active_trees_ind:
                            self.active_trees_ind.append(tree_ind+1)
                        for i in curr_tree.graph.endpoint_ind:
                            if curr_tree.graph[i].rank == ENDPOINT:
                                unblock = i - len(next_tree.graph.origin_ind)
                                next_tree.graph.unblock_origin(unblock)
                                break

    def start_thread(self):
        self.tree_thr = thr.Thread(target=self.run)
        self.tree_thr.start()

    def get_path(self):
        out_path = []
        paths_lens = []
        for tree_ind in self.active_trees_ind:
            curr_tree = self.trees[tree_ind]
            if curr_tree.dist_reached:
                last_node = sorted(curr_tree.path_ends, key=lambda x:curr_tree.graph[x].pos[-1])[0]
                path = curr_tree.get_path(last_node)
                out_path += path[::-1]
                paths_lens.append(len(path))
        return out_path, paths_lens

    def print_report(self):
        for tree_ind in self.active_trees_ind:
            curr_tree = self.trees[tree_ind]
            print("tree", tree_ind)
            print(curr_tree.graph.available_parents)
            print(curr_tree.graph.opened_nodes)
            print(curr_tree.graph.closed_nodes)
            for i in range(curr_tree.graph.node_num):
                if curr_tree.graph[i].rank != SLAVE:
                    print("node", i, curr_tree.graph[i].rank_str)
            if curr_tree.dist_reached:
                last_node = sorted(curr_tree.path_ends, key=lambda x: curr_tree.graph[x].pos[-1])[0]
                path = curr_tree.get_path(last_node)
                print(path[::-1])

