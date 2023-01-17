import pygame as pg


def range_cut(mi, ma, val):
    return min(ma, max(mi, val))


def convert_range(new_min, new_max, old_min, old_max, old_value):
    old_range = (old_max - old_min)
    new_range = (new_max - new_min)
    return (((old_value - old_min) * new_range) / old_range) + new_min


class NoCvMatSet(Exception):
    """Raised when no cv mat set for object of class Mat"""
    pass


class Sprite:
    def __init__(self, par_surf, /,
                 x=0,
                 y=0,
                 width=0,
                 height=0,
                 func=lambda *args: args,
                 color=(100, 100, 100)):
        self.func = func
        self.par_surf = par_surf
        self.ps_width, self.ps_height = par_surf.width, par_surf.height
        self.x = int(x * self.ps_width)
        self.y = int(y * self.ps_height)
        self.width = int(width * self.ps_width)
        self.height = int(height * self.ps_height)
        self.color = color
        if width != 0 and height != 0:
            self.surf = pg.Surface((self.width, self.height))
            self.surf.fill(self.color)
            self.rect = par_surf.add_object(self)

    def pressed(self, *args):
        pass

    def dragged(self, *args):
        pass

    def hover(self, *args):
        pass

    def update(self):
        pass


class Button(Sprite):
    def __init__(self, par_surf, /, **kwargs):
        super().__init__(par_surf, **kwargs)

    def pressed(self, *args):
        self.func(args)


class Text(Sprite):
    def __init__(self, par_surf, /,
                 inp_text=lambda *args: "your text",
                 font='serif',
                 font_size=10, **kwargs):
        super().__init__(par_surf, **kwargs)
        self.inp_text = inp_text
        self.text = inp_text
        self.text = pg.font.SysFont(font, int(font_size * self.ps_height / 500))
        self.surf = self.text.render(self.inp_text(), False, self.color)
        self.rect = par_surf.add_object(self)


    def pressed(self, *args):
        self.func(args)

    def update(self):
        self.surf = self.text.render(self.inp_text(), False, self.color)


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

        pg.draw.rect(self.surf, self.color, (0, 0, self.width, self.height), border_radius=self.height // 2)
        pg.draw.circle(self.surf, (255, 255, 255), (self.slider_x, self.slider_y), self.slider_rad)

    def set_val(self, val):
        self.slider_x = convert_range(self.slider_rad, self.width - self.slider_rad, self.min, self.max, val)

    def dragged(self, *args):
        self.slider_x = range_cut(self.slider_rad, self.width - self.slider_rad, args[0])
        self.val = convert_range(self.min, self.max, self.slider_rad, self.width - self.slider_rad, self.slider_x)
        self.func(self.val)

    def update(self):
        pg.draw.rect(self.surf, self.color, (0, 0, self.width, self.height), border_radius=self.height // 2)
        pg.draw.circle(self.surf, (255, 255, 255), (self.slider_x, self.slider_y), self.slider_rad)


class Mat:
    def __init__(self, par_surf, /,
                 func=lambda *args: args,
                 x=0,
                 y=0,
                 width=0,
                 height=0,
                 cv_mat_stream=None):
        self.par_surf = par_surf
        self.ps_width, self.ps_height = par_surf.width, par_surf.height
        self.x = int(x * self.ps_width)
        self.y = int(y * self.ps_height)
        self.width = int(width * self.ps_width)
        self.height = int(height * self.ps_height)
        self.func = lambda *args: args
        self.is_mat_stream = False
        self.last_hover_pos = (0, 0)
        self.is_pressed = False

        self.func = func

        if cv_mat_stream:
            self.cv_mat_stream = cv_mat_stream
        else:
            raise NoCvMatSet
        self.rect = par_surf.add_object(self)

    @property
    def surf(self):
        mat = self.cv_mat_stream()
        if self.width != 0 and self.height != 0:
            surf = pg.transform.flip(pg.transform.scale(pg.transform.rotate(pg.surfarray.make_surface(mat), -90),
                                                        (self.width, self.height)), 1, 0)
        else:
            surf = pg.transform.flip(pg.transform.rotate(pg.surfarray.make_surface(mat), -90), 1, 0)
        return surf

    def update(self):
        self.func(self.last_hover_pos, self.is_pressed)

    def pressed(self, *args):
        self.last_hover_pos = args
        self.is_pressed = True

    def dragged(self, *args):
        self.is_pressed = True
        self.last_hover_pos = args
        pass

    def hover(self, *args):
        self.last_hover_pos = args
        self.is_pressed = False
