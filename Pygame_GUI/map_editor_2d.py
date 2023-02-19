from Pygame_GUI.Objects import *


class MapEditor2d(Sprite):
    def __init__(self, par_surf, /, map_shape=None, **kwargs):
        super().__init__(par_surf, **kwargs)
        self.map_shape = np.array(map_shape)
        self.discrete_per_pixel_width = self.map_shape[0] / self.width
        self.discrete_per_pixel_height = self.map_shape[1] / self.height
        self.discrete_wh = self.width // self.map_shape[0], self.height // self.map_shape[1]
        self.bin_map = np.zeros(self.map_shape).astype(np.uint8)
        self.set_mode = 1
        self.origin = [100, 100]
        self.end_point = [300, 300]
        self.brush = 1

    def set_mode_wall(self, *args, **kwargs):
        self.set_mode = 1

    def set_mode_origin(self, *args, **kwargs):
        self.set_mode = 2

    def set_mode_point(self, *args, **kwargs):
        self.set_mode = 3

    def change_brush_size(self, val):
        self.brush = int(val)
        print(val)

    def fill_map(self, x, y, setter=1):
        bs = self.brush
        for di in range(-bs, bs):
            for dj in range(-bs, bs):
                i, j = x + di, y + dj
                if di ** 2 + dj ** 2 < bs ** 2 and self.map_shape[0] > i >= 0 and self.map_shape[1] > j >= 0:
                    self.bin_map[i, j] = setter

    def release(self, *args, **kwargs):
        # self.update_map()
        pass

    def pressed(self, *args, **kwargs):
        pos = kwargs["mouse_pos"]
        x = int(pos[0] * self.discrete_per_pixel_width)
        y = int(pos[1] * self.discrete_per_pixel_height)
        if kwargs["btn_id"] == 1:
            if not self.bin_map[x, y].any():
                if self.set_mode == 2:
                    self.origin = [x, y]
                elif self.set_mode == 3:
                    self.end_point = [x, y]
                else:
                    self.fill_map(x, y)
            else:
                print("no")
        else:
            self.fill_map(x, y, 0)

    def dragged(self, *args, **kwargs):
        if self.set_mode != 1:
            return
        setter = 0
        if kwargs["btn_id"] == 1:
            setter = 1
        pos = kwargs["mouse_pos"]
        x = int(pos[0] * self.discrete_per_pixel_width)
        y = int(pos[1] * self.discrete_per_pixel_height)
        self.fill_map(x, y, setter)

    def discrete_rect(self, i, j):
        x = i * self.discrete_wh[0]
        y = j * self.discrete_wh[1]
        return (x, y, *self.discrete_wh)

    def draw(self):
        out = self.bin_map == 0
        self.surface.blit(
            pg.transform.scale(pg.surfarray.make_surface(out * 255), (self.width, self.height)), (0, 0))

        pg.draw.circle(self.surface, (0, 255, 0), self.origin, 10)
        pg.draw.circle(self.surface, (255, 0, 0), self.end_point, 10)
