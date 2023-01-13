from RRT import *

class RRT_connect():
    def __init__(self, /,
                 points=None,
                 bin_map=None,
                 growth_factor=100,
                 e=0.04,
                 end_dist=100,
                 rrt_star_rad=20):
        self.points = points
        self.points_num = len(points)
        self.trees = []
        for i in points:
            self.trees.append(RRT(start_point=i,
                                  bin_map=bin_map,
                                  growth_factor=growth_factor,
                                  e=e,
                                  end_dist=end_dist,
                                  rrt_star_rad=rrt_star_rad))
            self.trees[-1].step()

    def step(self):
        for i in range(self.points_num):
            self.trees[i].step()
            for j in range(self.points_num):


