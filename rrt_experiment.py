import math

import cv2

from Pygame_GUI.Objects import *
from Pygame_GUI.Screen import Screen
from Pygame_GUI.Space3D.object_3d import *
from Pygame_GUI.map_editor_2d import MapEditor2d
from pathfinding.vanilla_rrt.rrt_tree import TreeRRT


def generate_kernel(rad):
    out = np.zeros((1 + rad * 2, 1 + rad * 2))
    x, y = rad, rad
    init = 60
    for di in range(-rad, rad + 1):
        for dj in range(-rad, rad + 1):
            i, j = x + di, y + dj
            out[i, j] = init - math.sqrt(di ** 2 + dj ** 2)
    return out


def generate_kernel_2(rad):
    out = np.zeros((1 + rad * 2, 1 + rad * 2))
    x, y = rad, rad
    for di in range(-rad, rad + 1):
        for dj in range(-rad, rad + 1):
            i, j = x + di, y + dj
            if di ** 2 + dj ** 2 < rad ** 2:
                out[i, j] = -1
    out[rad, rad] = rad**2
    return out


class RrtTestGui:
    def __init__(self, width, height):
        self.map_shape = (400, 400)
        self.width, self.height = width, height
        self.screen = Screen(900, 1100)
        self.screen.init()

        self.map_editor = self.screen.sprite(MapEditor2d, "MapEditor", x=0.0, y=0, width=1, height=0.8,
                                             color=(255, 255, 255), map_shape=self.map_shape)
        self.screen.sprite(Button, "set_origin", x=0.24, y=0.82, width=0.12, height=0.04, color=(0, 255, 0),
                           func=self.map_editor.set_mode_origin)
        self.screen.sprite(Button, "set_point", x=0.38, y=0.82, width=0.12, height=0.04, color=(0, 0, 255),
                           func=self.map_editor.set_mode_point)
        self.screen.sprite(Button, "set_wall", x=0.52, y=0.82, width=0.12, height=0.04, color=(132, 31, 39),
                           func=self.map_editor.set_mode_wall)
        self.screen.sprite(Button, "generate_field", x=0.80, y=0.82, width=0.12, height=0.04, color=(132, 180, 39),
                           func=self.generate_field)
        self.screen.sprite(Button, "run_rrt", x=0.66, y=0.82, width=0.12, height=0.08, color=(255, 0, 255),
                           func=self.run_rrt, image="Pygame_GUI/sprite_images/pathfinding.png")

        self.screen.sprite(Text, "set_origin_label", x=0.24, y=0.86, inp_text=lambda: "set_origin", font='serif',
                           font_size=10)
        self.screen.sprite(Text, "set_point_label", x=0.38, y=0.86, inp_text=lambda: "set_point", font='serif',
                           font_size=10)
        self.screen.sprite(Text, "set_wall_label", x=0.52, y=0.86, inp_text=lambda: "set_wall", font='serif',
                           font_size=10)
        self.screen.sprite(Slider, "curr_time", min=0, max=20, x=0.03, y=0.91,
                           width=0.94, height=0.05, color=(150, 160, 170),
                           func=self.screen["MapEditor"].change_brush_size)

        self.old_pressed_keys = []
        self.old_mouse_pos = [0, 0]
        self.screen.add_fps_indicator()

        self.tree_running = False

        self.bin_map = np.zeros(self.map_shape).astype(np.uint8)

    def run_rrt(self, *args, **kwargs):
        start_point = self.map_editor.origin
        end_point = self.map_editor.end_point

        self.tree = TreeRRT(start_point=np.array(start_point), end_point=np.array(end_point), bin_map=self.bin_map)
        self.tree.step()
        self.tree_running = True

    def generate_field(self, *args, **kwargs):
        # Reading the image
        image = (self.map_editor.bin_map.astype(np.uint8) == 1).astype(np.uint8)
        image_neg = (self.map_editor.bin_map.astype(np.uint8) != 1).astype(np.uint8)

        # Creating the kernel(2d convolution matrix)
        kernel1 = generate_kernel(70) / 5

        img = cv2.filter2D(src=image, ddepth=-1, kernel=kernel1)
        ksize = 100
        #blur_image = cv2.blur(image_neg*255, (ksize, ksize))
        blur_image = cv2.filter2D(src=image_neg, ddepth=-1, kernel=generate_kernel_2(50))
        out_image = ((blur_image) != image) * image_neg
        cv2.imshow('Original', image_neg*255)
        #cv2.imshow('blur_image', (blur_image)*10)
        cv2.imshow('Special interest', blur_image)



        cv2.waitKey()
        cv2.destroyAllWindows()

    def run(self):
        while self.screen.running:
            self.screen.step()


trgui = RrtTestGui(WIDTH, HEIGHT)
trgui.run()
