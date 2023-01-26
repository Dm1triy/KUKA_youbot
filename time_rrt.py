from Pygame_GUI.Screen import Screen
from Pygame_GUI.Space3D.Space3D import Space3D
from Pygame_GUI.Space3D.object_3d import *
from Pygame_GUI.Objects import *
from Pygame_GUI.constants import *


class TimeRrtGui:
    def __init__(self, width, height):
        self.map_shape = (100, 100, 100)
        self.width, self.height = width, height
        self.screen = Screen(self.width, self.height)
        self.space_3d = Space3D(self.screen, x=0, y=0, width=1, height=1)
        self.old_pressed_keys = []
        self.old_mouse_pos = [0, 0]
        self.space_3d.load_object_from_file("Pygame_GUI/Space3D/teapot.obj", (5, 0, 0))
        self.cube = (self.space_3d, [*np.array([(-1, -1, -1, 1), (1, -1, -1, 1), (-1, 1, -1, 1), (-1, -1, 1, 1),
                                                (1, 1, -1, 1), (1, -1, 1, 1),
                                                (-1, 1, 1, 1), (1, 1, 1, 1)]) / 2],
                     [[0, 1, 4, 2][::-1], [3, 6, 7, 5][::-1], [0, 3, 5, 1][::-1], [2, 4, 7, 6][::-1],
                      [1, 5, 7, 4][::-1], [2, 6, 3, 0][::-1]])
        self.space_3d.add_object(Axes(self.space_3d))
        Button(self.screen, x=0.9, y=0.01, width=0.08, height=0.06, color=(150, 255, 170), func=self.change_cam_mode)
        self.bin_map = np.zeros(self.map_shape).astype(np.uint8)
        #self.generate_random_map()

    def generate_random_map(self):
        rand_arr = np.random.rand(*self.map_shape)
        for i in range(self.map_shape[0]):
            for j in range(self.map_shape[1]):
                for k in range(self.map_shape[2]):
                    if rand_arr[i, j, k] > 0.999:
                        self.space_3d.add_object(Solid3D(*self.cube, (i-self.map_shape[0]/2, j-self.map_shape[1]/2, k-self.map_shape[2]/2)))
                        #self.space_3d.add_object(Solid3D(*self.cube, (i, j, k)))
                        self.bin_map[i, j, k] = 1
                        print([i, j, k])
            print(i)

    def run(self):
        while self.screen.running:
            self.screen.step()

    def change_cam_mode(self, *args):
        self.space_3d.camera.mode = not self.space_3d.camera.mode
        self.space_3d.camera.reset()


trgui = TimeRrtGui(WIDTH, HEIGHT)
trgui.run()
