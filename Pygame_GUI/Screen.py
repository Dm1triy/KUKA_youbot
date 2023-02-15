import pygame as pg
from Pygame_GUI.Objects import *


class Screen:
    def __init__(self, width, height, fps=60):
        self.running = True
        self.objects = dict()
        self.objects_order = []
        self.raw_draw = dict()
        self.blit_arr = []
        self.width, self.height = width, height
        self.screen = None
        self.pressed_obj_ind = None
        self.fps = 0
        self.pressed_keys = []
        self.curr_obj = None
        self.clock = pg.time.Clock()
        self.max_fps = fps
        self.bg_color = (0,0,0)

        self.mouse_delta = [0, 0]
        self.process_btn_clk = 0
        self.pressed_obj = None
        self.released_obj = None
        self.old_mouse_pos = [0, 0]
        self.mouse_wheel_pos = 0
        self.mouse_pos = [0, 0]
        self.mouse_state = [0, 0]

    def __getitem__(self, item):
        if item in self.objects:
            return self.objects[item]
        else:
            raise AttributeError("No object named", item)

    def add_fps_indicator(self, x=0, y=0, font=10):
        self.sprite(Text, "fps", x=x, y=y, inp_text=self.get_fps, font='serif', font_size=font)

    def init(self):
        pg.init()
        self.screen = pg.display.set_mode((self.width, self.height))

    def step(self):
        if self.running:
            self.handle_events()
            self.update_objects()
            self.draw_objects()
            pg.display.update()
            pg.display.flip()
            self.clock.tick(self.max_fps)
            self.fps = self.clock.get_fps()

    def run(self):
        while self.running:
            self.step()

    def end(self):
        self.running = False
        pg.quit()

    def get_fps(self):
        return round(self.fps, 2)

    def handle_events(self):
        self.mouse_pos = pg.mouse.get_pos()
        for obj_name in self.objects_order[::-1]:
            obj = self.objects[obj_name]
            if obj.rect.collidepoint(self.mouse_pos):
                self.curr_obj = obj
                break
            self.curr_obj = None

        for event in pg.event.get():
            if event.type == pg.QUIT:
                self.running = False
                pg.quit()
                return
            elif event.type == pg.KEYDOWN:
                if event.key == pg.K_ESCAPE:
                    self.running = False
                    pg.quit()
                    return
                else:
                    self.pressed_keys.append(event.key)
            elif event.type == pg.KEYUP:
                if event.key in self.pressed_keys:
                    self.pressed_keys.pop(self.pressed_keys.index(event.key))
            elif event.type == pg.MOUSEWHEEL:
                self.mouse_wheel_pos += event.y
            elif event.type == pg.MOUSEBUTTONDOWN:
                self.mouse_state = [event.button, 1]
                self.pressed_obj = self.curr_obj
            elif event.type == pg.MOUSEBUTTONUP:
                self.mouse_state = [event.button, 0]
                self.released_obj = self.curr_obj
        self.mouse_delta = [self.old_mouse_pos[0] - self.mouse_pos[0], self.old_mouse_pos[1] - self.mouse_pos[1]]
        self.old_mouse_pos = self.mouse_pos[:]

    def add_object(self, obj):
        self.objects[obj.name] = obj
        self.objects_order.append(obj.name)

    def add_raw(self, func, name, *args):
        self.raw_draw[name] = [func, args]

    def add_blit(self, img, pos):
        self.blit_arr.append((img, pos))
    def update_objects(self):
        if self.pressed_obj:
            pomop = self.pressed_obj.convert_to_local(self.mouse_pos)
            self.pressed_obj.pressed(mouse_pos=pomop, btn_id=self.mouse_state[0])
            self.pressed_obj = None
        if self.released_obj:
            pomop = self.released_obj.convert_to_local(self.mouse_pos)
            self.released_obj.release(mouse_pos=pomop, btn_id=self.mouse_state[0])
            self.released_obj = None
        elif self.curr_obj:
            curr_obj_mouse_pos = self.curr_obj.convert_to_local(self.mouse_pos)
            if (self.mouse_delta[0] or self.mouse_delta[1]) and self.mouse_state[1]:
                self.curr_obj.dragged(mouse_pos=curr_obj_mouse_pos,
                                      btn_id=self.mouse_state[0],
                                      mouse_delta=self.mouse_delta)
            else:
                self.curr_obj.hover(mouse_pos=curr_obj_mouse_pos)

        for obj in self.objects_order:
            self.objects[obj].update()

    def draw_objects(self):
        self.screen.fill(self.bg_color)
        for img, pos in self.blit_arr:
            self.screen.blit(img, pos)
        self.blit_arr = []
        for obj_name in self.objects_order:
            self.objects[obj_name].draw()
        for func, args in self.raw_draw.values():
            func(self.screen, *args)
        self.raw_draw = dict()

    def sprite(self, sprite_type, name, **kwargs):
        self.add_object(sprite_type(self, name=name, **kwargs))
        return self.objects[name]
