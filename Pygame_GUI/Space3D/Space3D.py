import pygame as pg
from Pygame_GUI.Space3D.object_3d import *
from Pygame_GUI.Space3D.camera import *
from Pygame_GUI.Space3D.projection import *
from Pygame_GUI.Space3D.fast_math import *


class Space3D:
    def __init__(self, par_surf, /,
                 func=lambda *args: args,
                 x=0,
                 y=0,
                 width=0,
                 height=0):
        self.par_surf = par_surf
        self.ps_width, self.ps_height = par_surf.width, par_surf.height
        self.x = int(x * self.ps_width)
        self.y = int(y * self.ps_height)
        self.width, self.height = int(width * self.ps_width), int(height * self.ps_height)
        self.h_width, self.h_height = self.width // 2, self.height // 2
        self.color_mat = np.zeros((self.width, self.height, 3), dtype=np.uint32)
        self.depth_mat = np.zeros((self.width, self.height), dtype=np.float32)
        self.func = lambda *args: args
        self.last_hover_pos = (0, 0)
        self.last_mouse_wheel = 0
        self.is_pressed = False
        self.mouse_sense = 0.1
        self.old_pressed_keys = []
        self.min_dist = 3

        self.func = func
        #self.get_object_from_file("Pygame_GUI/Space3D/t_34_obj.obj")

        self.camera = Camera(self, [0, 0, -10])
        self.update_camera_pos = True
        self.projection = Projection(self)
        self.operating_surf = pg.Surface((self.width, self.height))

        self.all_obj = []#Axes(self)]
        self.rect = par_surf.add_object(self)

    def load_object_from_file(self, filename):
        vertex, faces = [], []
        with open(filename) as f:
            for line in f:
                if line.startswith('v '):
                    vertex.append([float(i) for i in line.split()[1:]] + [1])
                elif line.startswith('f'):
                    faces_ = line.split()[1:]
                    faces.append([int(face_.split('/')[0]) - 1 for face_ in faces_])
        self.all_obj.append(Solid3D(self, vertex, faces))

    @property
    def surf(self):
        self.color_mat = np.zeros((self.width, self.height, 3), dtype=np.int32)
        self.depth_mat = np.zeros((self.width, self.height), dtype=np.float64)
        self.operating_surf.fill(pg.Color('darkslategray'))
        for i in self.all_obj:
            i.draw()
        #return self.operating_surf
        #maybe blit_array
        return pg.surfarray.make_surface(self.color_mat)

    def add_object(self, vertices, faces):
        self.all_obj.append(Solid3D(self, vertices, faces))


    def update(self):
        if self.rect.collidepoint(pg.mouse.get_pos()):
            self.camera.control()
            self.update_keys()
        self.func(self.last_hover_pos, self.is_pressed)

    def pressed(self, *args):
        self.last_hover_pos = args
        self.is_pressed = True

    def dragged(self, *args):
        self.is_pressed = True
        if args[2] == 1:
            rotation = (self.last_hover_pos[0] - args[0]) * self.mouse_sense, (
                    self.last_hover_pos[1] - args[1]) * self.mouse_sense, 0
        else:
            rotation = 0, 0, (self.last_hover_pos[0] - args[0]) * self.mouse_sense

        self.camera.control(rotation=rotation)
        self.last_hover_pos = args[:2]

    def hover(self, *args):
        self.last_hover_pos = args[:2]
        self.is_pressed = False
        delta = self.last_mouse_wheel - self.par_surf.mouse_wheel_pos
        self.last_mouse_wheel = self.par_surf.mouse_wheel_pos
        if self.camera.mode == 1:
            self.camera.control(transition=np.array([0, 0, -delta, 0]))

    def released(self, *args):
        pass

    def clicked(self, *args):
        pass

    def slip(self, *args):
        self.camera.control(transition=np.array([0, 0, 0, 0]), rotation=[0, 0, 0])

    def update_keys(self):
        pressed_keys = self.par_surf.pressed_keys[:]
        transition = np.array([0, 0, 0, 0])
        if pg.K_a in pressed_keys:
            transition[0] = -1
        if pg.K_d in pressed_keys:
            transition[0] = 1
        if pg.K_w in pressed_keys:
            transition[2] = 1
        if pg.K_s in pressed_keys:
            transition[2] = -1
        if pg.K_LSHIFT in pressed_keys:
            transition[1] = 1
        if pg.K_LCTRL in pressed_keys:
            transition[1] = -1
        if self.old_pressed_keys != pressed_keys:
            if self.camera.mode == 0:
                self.camera.control(transition=transition)
            self.old_pressed_keys = pressed_keys[:]
