from Pygame_GUI.Screen import Screen
from Pygame_GUI.Space3D.Space3D import Space3D
from Pygame_GUI.Space3D.object_3d import *
from Pygame_GUI.Objects import *
from pathfinding.RRT import RRT
import time


class MapEditor(Sprite):
    def __init__(self, par_surf, /, **kwargs):
        super().__init__(par_surf, **kwargs)
        self.map_shape = (30, 30, 100)
        self.discrete_per_pixel_width = self.map_shape[0] / self.width
        self.discrete_per_pixel_height = self.map_shape[1] / self.height
        self.discrete_wh = self.width // self.map_shape[0], self.height // self.map_shape[1]
        self.bin_map = np.zeros(self.map_shape).astype(np.uint8)
        self.time_range = [0, self.map_shape[-1]]
        self.curr_time = 0

    def change_curr_time(self, val):
        self.curr_time = int(val)

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
        setter = 0
        if kwargs["btn_id"] == 1:
            setter = 1
        pos = kwargs["mouse_pos"]
        x = int(pos[0] * self.discrete_per_pixel_width)
        y = int(pos[1] * self.discrete_per_pixel_height)
        self.bin_map[x, y, self.time_range[0]:self.time_range[1]] = setter

    def dragged(self, *args, **kwargs):
        setter = 0
        if kwargs["btn_id"] == 1:
            setter = 1
        pos = kwargs["mouse_pos"]
        x = int(pos[0] * self.discrete_per_pixel_width)
        y = int(pos[1] * self.discrete_per_pixel_height)
        self.bin_map[x, y, self.time_range[0]:self.time_range[1]] = setter

    def discrete_rect(self, i, j):
        x = i * self.discrete_wh[0]
        y = j * self.discrete_wh[1]
        return (x, y, *self.discrete_wh)

    def draw(self):
        pg.draw.rect(self.surface, self.color, self.rect)
        for i in range(self.map_shape[0]):
            for j in range(self.map_shape[1]):
                if self.bin_map[i, j, self.curr_time] == 1:
                    pg.draw.rect(self.surface, (0, 0, 0), self.discrete_rect(i, j))
