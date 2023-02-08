import pygame as pg

from Pygame_GUI.Space3D.object_3d import *
from Pygame_GUI.Space3D.camera import *
from Pygame_GUI.Space3D.projection import *
from Pygame_GUI.Space3D.fast_math import *
from Pygame_GUI.Objects import Sprite


class Space3D(Sprite):
    def __init__(self, par_surf, /,
                 real_width=WIDTH,
                 real_height=HEIGHT, **kwargs):
        super().__init__(par_surf, **kwargs)

        self.real_width, self.real_height = int(real_width), int(real_height)
        self.h_width, self.h_height = self.width // 2, self.height // 2

        self.workspace = (np.zeros((WIDTH, HEIGHT, 3), dtype=np.uint32),  # color
                          np.zeros((WIDTH, HEIGHT), dtype=float_bit),  # depth
                          np.zeros((WIDTH, HEIGHT), dtype=float_bit))  # priority

        self.last_drag_pos = (0, 0)
        self.last_mouse_wheel = 0

        self.mouse_sense = 0.1
        self.old_pressed_keys = []
        self.min_dist = 3

        self.camera = Camera(self, [0, 0, -10])
        self.projection = Projection(self)

        self.all_obj = []

    def load_object_from_file(self, filename, pos=None):
        vertex, faces = [], []
        with open(filename) as f:
            for line in f:
                if line.startswith('v '):
                    vertex.append([float(i) for i in line.split()[1:]] + [1])
                elif line.startswith('f'):
                    faces_ = line.split()[1:]
                    faces.append([int(face_.split('/')[0]) - 1 for face_ in faces_])
        self.all_obj.append(Solid3D(self, vertex, faces, pos))

    def clear_workspace(self):
        self.workspace = (np.zeros((WIDTH, HEIGHT, 3), dtype=np.uint32),  # color
                          np.zeros((WIDTH, HEIGHT), dtype=float_bit),  # depth
                          np.zeros((WIDTH, HEIGHT), dtype=float_bit))  # priority

    def draw(self):
        self.clear_workspace()
        for i in self.all_obj:
            i.vert_to_global()
        self.all_obj = sorted(self.all_obj, key=lambda x:np.linalg.norm(x.global_center_of_mass), reverse=True)
        for i in self.all_obj:
            i.draw(self.workspace)
        # maybe blit_array
        self.rect = self.surface.blit(
            pg.transform.scale(pg.surfarray.make_surface(self.workspace[0]), (self.width, self.height)), self.pos)

    def add_object(self, obj):
        self.all_obj.append(obj)

    def update(self):
        if self.rect.collidepoint(pg.mouse.get_pos()):
            self.camera.control()
            self.update_keys()

        delta = self.last_mouse_wheel - self.par_surf.mouse_wheel_pos
        self.last_mouse_wheel = self.par_surf.mouse_wheel_pos
        if self.camera.mode == 1 and delta:
            self.camera.control(transition=np.array([0, 0, -delta, 0]))

    def dragged(self, *args, **kwargs):
        if kwargs["btn_id"] == 1:
            rotation = kwargs["mouse_delta"][0] * self.mouse_sense, kwargs["mouse_delta"][1] * self.mouse_sense, 0
        else:
            rotation = 0, 0, kwargs["mouse_delta"][0] * self.mouse_sense

        self.camera.control(rotation=rotation)

    def slip(self, *args, **kwargs):
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
