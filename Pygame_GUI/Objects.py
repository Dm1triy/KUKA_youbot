import pygame as pg
import numpy as np


def range_cut(mi, ma, val):
    return min(ma, max(mi, val))


def convert_range(new_min, new_max, old_min, old_max, old_value):
    old_range = (old_max - old_min)
    new_range = (new_max - new_min)
    return (((old_value - old_min) * new_range) / old_range) + new_min


class NoCvMatSet(Exception):
    """Raised when no cv mat set for object of class Mat"""
    pass


class NoDrawFuncReloaded(Exception):
    """Raised when sprite draw func is not reloaded in child class"""
    pass


class Sprite:
    def __init__(self, par_surf, /,
                 name="Sprite",
                 x=0,
                 y=0,
                 width=0,
                 height=0,
                 func=lambda *args, **kwargs: args,
                 color=(100, 100, 100)):
        self.func = func
        self.name = name
        self.par_surf = par_surf
        self.surface = self.par_surf.screen
        self.ps_width, self.ps_height = par_surf.width, par_surf.height
        self.x = int(x * self.ps_width)
        self.y = int(y * self.ps_height)
        self.pos = self.x, self.y
        self.width = int(width * self.ps_width)
        self.height = int(height * self.ps_height)
        self.color = color
        self.rect = pg.Rect(self.x, self.y, self.width, self.height)

    def pressed(self, *args, **kwargs):
        pass

    def dragged(self, *args, **kwargs):
        pass

    def release(self, *args, **kwargs):
        pass

    def hover(self, *args, **kwargs):
        pass

    def update(self):
        pass

    def convert_to_local(self, coords):
        return coords[0] - self.x, coords[1] - self.y

    def draw(self):
        raise NoDrawFuncReloaded


class Button(Sprite):
    def __init__(self, par_surf, /, image="", radius=10, **kwargs):
        super().__init__(par_surf, **kwargs)
        self.radius = radius
        if image:

            if isinstance(image, str):
                self.have_image = 1
                self.image = pg.transform.scale(pg.image.load(image), (self.width, self.height))

            else:
                self.have_image = len(image)
                self.image_arr = []
                for img in image:
                    self.image_arr.append(pg.transform.scale(pg.image.load(img), (self.width, self.height)))
                self.image = self.image_arr[0]
                self.curr_img = 0
        else:
            self.have_image = 0

    def pressed(self, *args, **kwargs):
        self.func(*args, **kwargs)

    def draw(self):
        if self.have_image == 1:
            self.surface.blit(self.image, self.rect)
        elif self.have_image > 1:
            self.surface.blit(self.image_arr[self.curr_img], self.rect)
        else:
            pg.draw.rect(self.surface, self.color, self.rect, border_radius=self.radius)


class Text(Sprite):
    def __init__(self, par_surf, /,
                 inp_text=lambda *args: "your text",
                 color=(255, 255, 255),
                 font='serif',
                 font_size=10, **kwargs):
        super().__init__(par_surf, color=color, **kwargs)
        self.inp_text = inp_text
        self.text = pg.font.SysFont(font, int(font_size * self.ps_height / 500))
        self.txt_render = self.text.render(str(self.inp_text()), True, self.color)

    def pressed(self, *args, **kwargs):
        self.func(args, **kwargs)

    def update(self):
        self.txt_render = self.text.render(str(self.inp_text()), True, self.color)

    def draw(self):
        self.rect = self.surface.blit(self.txt_render, self.pos)


class Slider(Sprite):
    def __init__(self, par_surf, /,
                 slider_color=(255, 255, 255),
                 min=0,
                 max=100,
                 val=None, **kwargs):
        super().__init__(par_surf, **kwargs)
        self.slider_color = slider_color
        self.min = min
        self.max = max
        self.slider_rad = self.height // 2
        self.slider_y = self.slider_rad

        if val:
            self.val = val
        else:
            self.val = self.min

        self.slider_x = convert_range(self.slider_rad, self.width - self.slider_rad, self.min, self.max, self.val)

    def set_val(self, val):
        self.slider_x = convert_range(self.slider_rad, self.width - self.slider_rad, self.min, self.max, val)

    def dragged(self, *args, **kwargs):
        pos = kwargs["mouse_pos"]
        self.slider_x = range_cut(self.slider_rad, self.width - self.slider_rad, pos[0])
        self.val = convert_range(self.min, self.max, self.slider_rad, self.width - self.slider_rad, self.slider_x)
        self.func(self.val)

    def update(self):
        pg.draw.rect(self.surface, self.color, (0, 0, self.width, self.height), border_radius=self.height // 2)
        pg.draw.circle(self.surface, (255, 255, 255), (self.slider_x, self.slider_y), self.slider_rad)

    def draw(self):
        self.slider_x = convert_range(self.slider_rad, self.width - self.slider_rad, self.min, self.max, self.val)
        pg.draw.rect(self.surface, self.color, (self.x, self.y, self.width, self.height),
                     border_radius=self.height // 2)
        pg.draw.circle(self.surface, (255, 255, 255), (self.x + self.slider_x, self.y + self.slider_y), self.slider_rad)


class Mat(Sprite):
    def __init__(self, par_surf, /,
                 cv_mat_stream=None,
                 **kwargs):

        if cv_mat_stream:
            self.cv_mat_stream = cv_mat_stream
        else:
            raise NoCvMatSet
        super().__init__(par_surf, **kwargs)
        self.is_mat_stream = False
        self.last_hover_pos = (0, 0)
        self.is_pressed = False

    def draw(self):
        mat = self.cv_mat_stream()
        if self.width != 0 and self.height != 0:
            self.rect = self.surface.blit(pg.transform.flip(
                pg.transform.scale(pg.transform.rotate(pg.surfarray.make_surface(mat), -90), (self.width, self.height)),
                1, 0), self.pos)
        else:
            self.rect = self.surface.blit(
                pg.transform.flip(pg.transform.rotate(pg.surfarray.make_surface(mat), -90), 1, 0),
                self.pos)

    def update(self):
        self.func(mouse_pos=self.last_hover_pos, btn_id=self.is_pressed)

    def pressed(self, *args, **kwargs):
        self.last_hover_pos = kwargs["mouse_pos"]
        self.is_pressed = kwargs["btn_id"]

    def dragged(self, *args, **kwargs):
        self.last_hover_pos = kwargs["mouse_pos"]
        self.is_pressed = kwargs["btn_id"]
        pass

    def hover(self, *args, **kwargs):
        self.last_hover_pos = kwargs["mouse_pos"]
        self.is_pressed = False
