import pygame as pg


class Screen:
    def __init__(self, width, height, fps=60):
        pg.init()
        self.running = True
        self.objects = []
        self.width, self.height = width, height
        self.screen = pg.display.set_mode((width, height))
        self.pressed_obj_ind = None
        self.fps = 0
        self.pressed_keys = []
        self.curr_obj = None
        self.clock = pg.time.Clock()
        self.max_fps = fps

        self.mouse_delta = [0,0]
        self.process_btn_clk = 0
        self.pressed_obj = None
        self.old_mouse_pos = [0, 0]
        self.mouse_wheel_pos = 0
        self.mouse_pos = [0, 0]
        self.mouse_state = [0, 0]

    def step(self):
        if self.running:
            self.handle_events()
            self.update_object()
            pg.display.update()
            pg.display.flip()
            self.clock.tick(self.max_fps)
            self.fps = self.clock.get_fps()
            #print(self.clock.get_fps())

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
        for obj_ind in range(len(self.objects) - 1, -1, -1):
            obj = self.objects[obj_ind]
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
        self.mouse_delta = [self.old_mouse_pos[0] - self.mouse_pos[0], self.old_mouse_pos[1] - self.mouse_pos[1]]
        self.old_mouse_pos = self.mouse_pos[:]

    def add_object(self, obj):
        self.objects.append(obj)
        return self.screen.blit(obj.surf, (obj.x, obj.y))

    def update_object(self):
        if self.pressed_obj:
            self.pressed_obj.pressed(*self.mouse_pos, self.mouse_state[0])
            self.pressed_obj = None
        elif (self.mouse_delta[0] or self.mouse_delta[1]) and self.mouse_state[1]:
            self.curr_obj.dragged(*self.mouse_pos, *self.mouse_delta, self.mouse_state[0])
        if self.curr_obj:
            self.curr_obj.hover(*self.mouse_pos)

        for obj in self.objects:
            obj.update()
            obj.rect = self.screen.blit(obj.surf, (obj.x, obj.y))

