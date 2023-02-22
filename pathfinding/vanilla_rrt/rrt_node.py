import numpy as np
from pathfinding.vanilla_rrt.constants import *

class Node:
    def __init__(self, index, parent, pos, /, dist_to_origin=float('inf'), rank=SLAVE, children=None):

        self.pos = pos
        self.parent = parent

        self.dist_to_origin = dist_to_origin
        self.index = index
        self.rank = rank


        if children:
            for i in children:
                assert isinstance(i, Node)
            self.children = children
        else:
            self.children = []

        if parent != None:
            assert isinstance(parent, Node)
            self.dist_to_parent = np.linalg.norm(parent.pos - pos)
        else:
            self.dist_to_parent = float('inf')



    def __str__(self):
        return f"ind: {str(self.index)}\tparent: {str(self.parent.index)};\tpos: {str(self.pos)};\tchildren: {str(self.children)};\trank: {str(self.rank)};\tdist_to_origin: {str(self.dist_to_origin)}"

    def add_child(self, child):
        assert isinstance(child, Node)
        if child not in self.children:
            self.children.append(child)

    def del_child(self, child):
        assert isinstance(child, Node)
        if child in self.children:
            self.children.remove(child)

    def new_parent(self, n_parent):
        if self.parent != None:
            self.parent.del_child(self)
        self.parent = n_parent
        n_parent.add_child(self)
        self.dist_to_parent = np.linalg.norm(n_parent.pos-self.pos)
        self.rebalance()

    def rebalance(self):
        if self.parent != None:
            self.dist_to_origin = self.parent.dist_to_origin + self.dist_to_parent
        else:
            self.dist_to_origin = float("inf")
        for i in self.children:
            i.rebalance()



