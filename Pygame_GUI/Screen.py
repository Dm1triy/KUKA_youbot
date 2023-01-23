import pygame as pg


class Screen:
    def __init__(self, width, height, fps=60):
        pg.init()
        self.running = True
        self.objects = []
        self.width, self.height = width, height
        self.screen = pg.display.set_mode((width, height))
        self.pressed_obj_ind = None
        self.fps = 24
        self.pressed_keys = []
        self.mouse_wheel_pos = 0
        self.mouse_pos = [0, 0]
        self.old_mouse_pos = [0, 0]
        self.active_obj = {"obj": None, "event": None, "args": None}
        self.curr_obj = None
        self.last_mouse_btn = 0
        self.clock = pg.time.Clock()
        self.max_fps = fps
        self.fps = 0

    def step(self):
        if self.running:
            self.update_object()
            pg.display.update()
            pg.display.flip()
            self.handle_events()
            self.clock.tick(self.max_fps)
            self.fps = self.clock.get_fps()
            print(self.clock.get_fps())

    def run(self):
        while self.running:
            self.step()

    def end(self):
        self.running = False
        pg.quit()

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
            if self.curr_obj:
                if self.active_obj["event"] not in ("pressed", "dragged"):
                    self.active_obj["obj"] = self.curr_obj
                    self.active_obj["event"] = "hover"
                    self.active_obj["args"] = self.local_mouse_state(self.curr_obj)

                elif self.active_obj["event"] in ["dragged"]:
                    if self.active_obj["obj"] == self.curr_obj and self.old_mouse_pos != self.mouse_pos:
                        self.active_obj["event"] = "dragged"
                        self.active_obj["args"] = *self.local_mouse_state(obj), self.last_mouse_btn

                if event.type == pg.MOUSEBUTTONUP:
                    self.active_obj["event"] = "released"
                elif event.type == pg.MOUSEBUTTONDOWN:
                    self.active_obj["obj"] = self.curr_obj
                    self.active_obj["event"] = "pressed"
                    self.active_obj["args"] = *self.local_mouse_state(obj), event.button
                    self.last_mouse_btn = event.button
            else:
                self.active_obj = {"obj": None, "event": None, "args": None}
        self.old_mouse_pos = self.mouse_pos[:]

    def add_object(self, obj):
        self.objects.append(obj)
        return self.screen.blit(obj.surf, (obj.x, obj.y))

    def update_object(self):
        for obj in self.objects:
            obj.update()
            obj.rect = self.screen.blit(obj.surf, (obj.x, obj.y))
        obj, event, args = self.active_obj.values()
        if obj:
            if event == "pressed":
                self.active_obj["event"] = "dragged"
            getattr(obj, event)(*args)

    def local_mouse_state(self, obj):
        return pg.mouse.get_pos()[0] - obj.x, pg.mouse.get_pos()[1] - obj.y
