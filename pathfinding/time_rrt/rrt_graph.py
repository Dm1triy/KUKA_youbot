import numpy as np
from pathfinding.time_rrt.rrt_node import *


class Graph:
    def __init__(self, /, start_point=None, start_point_available=None, end_point=None):
        if not start_point_available:
            start_point_available = [True] * len(start_point)
        assert isinstance(start_point, np.ndarray)
        if not isinstance(end_point, np.ndarray):
            end_point = np.array(end_point)

        # first point
        self.graph = dict()
        self.graph[0] = Node(0, None, start_point[0], dist_to_origin=0, graph=self.graph)
        self.nodes = np.array([start_point[0]]).astype(np.float32)
        self.node_num = 1

        for i in range(1, len(start_point_available)):
            if start_point_available[i]:
                rank = ORIGIN
            else:
                rank = ORIGIN_BLOCKED
            self.add_node(i - 1, start_point[i], [], rank)

        if end_point.any():
            # end points
            self.target_ind = []
            self.target_ind.append(self.node_num)
            self.add_node(None, end_point[0], [], ENDPOINT_BLOCKED)
            for i in range(0, len(end_point)):
                self.target_ind.append(self.node_num)
                self.add_node(self.node_num-1, end_point[i], [], ENDPOINT_BLOCKED)

    def add_node(self, parent, pos, children=None, rank=SLAVE):
        """
        Adds node to graph
        :param parent: parent
        :param pos: node's coordinates
        :param children: node's children
        :param rank: node's rank
        :return:
        """
        nn = self.node_num
        if not isinstance(parent, type(None)):
            if not isinstance(parent, Node):
                parent = self.graph[parent]
            dist = parent.dist_to_origin + np.linalg.norm(parent.pos - pos)
            parent.add_child(nn)
            parent_ind = parent.index
        else:
            dist = float('inf')
            parent_ind = None

        if not children:
            children = []
        self.graph[nn] = Node(nn, parent_ind, pos, rank=rank, children=children,
                              dist_to_origin=dist, graph=self.graph)
        self.nodes = np.append(self.nodes, [pos], axis=0).astype(np.int32)
        self.node_num += 1
        return self.graph[nn]

    def __str__(self):
        out_str = "Graph:\n"
        for i in range(self.node_num):
            out_str += str(self.graph[i]) + "\n"
        return out_str

    def __len__(self):
        return self.node_num-1

    def __getitem__(self, item):
        if isinstance(item, Node):
            item = item.index
        if item == -1:
            item = self.node_num-1
        return self.graph[item]




