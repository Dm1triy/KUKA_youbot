from Pygame_GUI.Screen import Screen
from Pygame_GUI.Space3D.Space3D import Space3D
from Pygame_GUI.Space3D.object_3d import *
from Pygame_GUI.Objects import *
from pathfinding.RRT import RRT
import time


class MapEditor(Sprite):
    def __init__(self, par_surf, /, map_shape=None, **kwargs):
        super().__init__(par_surf, **kwargs)
        self.map_shape = map_shape
        self.discrete_per_pixel_width = self.map_shape[0] / self.width
        self.discrete_per_pixel_height = self.map_shape[1] / self.height
        self.discrete_wh = self.width // self.map_shape[0], self.height // self.map_shape[1]
        self.bin_map = np.zeros(self.map_shape).astype(np.uint8)
        self.time_range = [0, self.map_shape[-1]]
        self.curr_time = 0
        self.set_mode = 0
        self.origin = [1, 1, [0, 99]]
        self.end_point = [29, 29, [0, 99]]
        self.full_map = np.zeros(self.map_shape).astype(np.uint16)
        self.point_ind = 3
        self.points = dict()

    def set_mode_wall(self, *args, **kwargs):
        self.set_mode = 1

    def set_mode_origin(self, *args, **kwargs):
        self.set_mode = 2

    def set_mode_point(self, *args, **kwargs):
        self.set_mode = 3

    def change_curr_time(self, val):
        self.curr_time = int(val)

    def release(self, *args, **kwargs):
        self.update_map()

    def update_map(self):
        pass

    def input_ttl(self, *args, **kwargs):
        try:
            print("begin: ", end='')
            from_time = int(input())
            print("end: ", end='')
            to_time = int(input())
            if to_time < from_time:
                assert Exception
            else:
                self.time_range = [from_time, to_time]
        except:
            print("wrong format")

    def pressed(self, *args, **kwargs):
        pos = kwargs["mouse_pos"]
        x = int(pos[0] * self.discrete_per_pixel_width)
        y = int(pos[1] * self.discrete_per_pixel_height)
        if kwargs["btn_id"] == 1:
            if not self.full_map[x, y, self.time_range[0]:self.time_range[1]].any():

                if self.set_mode == 2:
                    self.full_map[self.origin[0], self.origin[1], self.origin[2][0]:self.origin[2][1]] = 0
                    self.origin = [x, y, self.time_range]
                    self.full_map[x, y, self.time_range[0]:self.time_range[1]] = self.set_mode
                elif self.set_mode == 3:
                    self.full_map[x, y, self.time_range[0]:self.time_range[1]] = self.point_ind
                    self.points[self.point_ind] = [x, y, self.time_range]
                    self.point_ind += 1
                    self.end_point = [x, y, self.time_range]
                else:
                    self.full_map[x, y, self.time_range[0]:self.time_range[1]] = 1
            else:
                print("no")
        else:
            to_del_ind = self.full_map[x, y, self.curr_time]
            if to_del_ind == 1:
                self.full_map[x, y, self.time_range[0]:self.time_range[1]] = 0
            elif to_del_ind and to_del_ind != 2:
                to_del_param = self.points[to_del_ind]
                del self.points[to_del_ind]
                self.full_map[to_del_param[0], to_del_param[1], to_del_param[2][0]:to_del_param[2][1]] = 0

    def dragged(self, *args, **kwargs):
        if self.set_mode != 0:
            return
        setter = 0
        if kwargs["btn_id"] == 1:
            setter = 1
        pos = kwargs["mouse_pos"]
        x = int(pos[0] * self.discrete_per_pixel_width)
        y = int(pos[1] * self.discrete_per_pixel_height)
        self.bin_map[x, y, self.time_range[0]:self.time_range[1]] = setter
        self.full_map[x, y, self.time_range[0]:self.time_range[1]] = setter

    def discrete_rect(self, i, j):
        x = i * self.discrete_wh[0]
        y = j * self.discrete_wh[1]
        return (x, y, *self.discrete_wh)

    def draw(self):
        pg.draw.rect(self.surface, self.color, self.rect)
        for i in range(self.map_shape[0]):
            for j in range(self.map_shape[1]):
                if self.full_map[i, j, self.curr_time] == 1:
                    pg.draw.rect(self.surface, (0, 0, 0), self.discrete_rect(i, j))
                elif self.full_map[i, j, self.curr_time] == 2:
                    pg.draw.rect(self.surface, (0, 255, 0), self.discrete_rect(i, j))
                elif self.full_map[i, j, self.curr_time] != 0:
                    pg.draw.rect(self.surface, (0, 0, 255), self.discrete_rect(i, j))
