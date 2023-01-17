import pygame as pg
from Pygame_GUI.Space3D.object_3d import *
from Pygame_GUI.Space3D.camera import *
from Pygame_GUI.Space3D.projection import *


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
        self.h_width, self.h_height = self.width//2, self.height//2
        self.func = lambda *args: args
        self.last_hover_pos = (0, 0)
        self.is_pressed = False

        self.func = func

        self.camera = Camera(self, [-5, 6, -55])
        self.projection = Projection(self)
        self.operating_surf = pg.Surface((self.width, self.height))


        self.all_obj = [Axes(self)]
        object = self.get_object_from_file('Pygame_GUI/Space3D/t_34_obj.obj')
        object.rotate_y(-math.pi / 4)

        self.all_obj.append(object)
        self.rect = par_surf.add_object(self)



    def get_object_from_file(self, filename):
        vertex, faces = [], []
        with open(filename) as f:
            for line in f:
                if line.startswith('v '):
                    vertex.append([float(i) for i in line.split()[1:]] + [1])
                elif line.startswith('f'):
                    faces_ = line.split()[1:]
                    faces.append([int(face_.split('/')[0]) - 1 for face_ in faces_])
        return Object3D(self, vertex, faces)

    @property
    def surf(self):
        self.operating_surf.fill(pg.Color('darkslategray'))
        for i in self.all_obj:
            i.draw()
        return self.operating_surf

    def update(self):
        self.camera.control()
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
