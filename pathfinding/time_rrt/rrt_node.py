import numpy as np


ORIGIN = 0
ORIGIN_BLOCKED = 1
SLAVE = 2
ENDPOINT = 3
ENDPOINT_BLOCKED = 4


class Node:
    def __init__(self, index, parent, pos, /, dist_to_origin=float('inf'), rank=SLAVE, children=None):
        self.pos = pos
        self.parent = parent
        if children:
            self.children = children
        else:
            self.children = []
        self.dist_to_origin = dist_to_origin
        self.index = index
        self.rank = rank

        if parent:
            self.dist_to_parent = np.linalg.norm(parent.pos-pos)

    def __str__(self):
        return f"ind: {str(self.index)}\tparent: {str(self.parent.index)};\tpos: {str(self.pos)};\tchildren: {str(self.children)};\trank: {str(self.rank)};\tdist_to_origin: {str(self.dist_to_origin)}"

    def add_child(self, child_ind):
        if isinstance(child_ind, Node):
            child_ind = child_ind.index
        assert child_ind not in self.children
        self.children.append(child_ind)

    def del_child(self, child_ind):
        if isinstance(child_ind, Node):
            child_ind = child_ind.index
        assert child_ind in self.children
        self.children.remove(child_ind)

    def new_parent(self, n_parent):
        if self.parent != None:
            self.parent.del_child(self)
        self.parent = n_parent.index
        n_parent.add_child(self)
        self.dist_to_parent = np.linalg.norm(n_parent.pos-self.pos)
        dist = abs(np.linalg.norm(self.pos - n_parent.pos))
        self.rebalance()

    def rebalance(self):
        node.dist_to_origin -= delta
        for i in node.children:
            self.rebalance(i, delta)



