from Pygame_GUI.Screen import Screen
from Pygame_GUI.Space3D.Space3D import Space3D
from Pygame_GUI.Space3D.object_3d import *
from Pygame_GUI.Objects import *
from Pygame_GUI.constants import *


class TimeRrtGui:
    def __init__(self, width, height):
        self.map_shape = (30, 30, 100)
        self.width, self.height = width, height
        self.screen = Screen(self.width, self.height)
        self.space_3d = Space3D(self.screen, x=0, y=0, width=1, height=1)
        self.old_pressed_keys = []
        self.old_mouse_pos = [0, 0]
        self.fps_text = Text(self.screen, x=0.1, y=0.01, inp_text=self.screen.get_fps, font='serif', font_size=10)
        # self.space_3d.load_object_from_file("Pygame_GUI/Space3D/teapot.obj", (5, 0, 0))
        self.cube = (self.space_3d, [*np.array([(-1, -1, -1, 1), (1, -1, -1, 1), (-1, 1, -1, 1), (-1, -1, 1, 1),
                                                (1, 1, -1, 1), (1, -1, 1, 1),
                                                (-1, 1, 1, 1), (1, 1, 1, 1)]) / 2],
                     [[0, 1, 4, 2][::-1], [3, 6, 7, 5][::-1], [0, 3, 5, 1][::-1], [2, 4, 7, 6][::-1],
                      [1, 5, 7, 4][::-1], [2, 6, 3, 0][::-1]])
        self.space_3d.add_object(Axes(self.space_3d))
        Button(self.screen, x=0.9, y=0.01, width=0.08, height=0.06, color=(150, 255, 170), func=self.change_cam_mode)
        self.bin_map = np.zeros(self.map_shape).astype(np.uint8)
        # self.generate_random_map()
        self.manual_map()
        # self.generate_map_3d_graphics(self.bin_map)

    def generate_random_map(self):
        rand_arr = np.random.rand(*self.map_shape)
        for i in range(self.map_shape[0]):
            for j in range(self.map_shape[1]):
                for k in range(self.map_shape[2]):
                    if rand_arr[i, j, k] > 0.999:
                        self.space_3d.add_object(Solid3D(*self.cube, (
                            i - self.map_shape[0] / 2, j - self.map_shape[1] / 2, k - self.map_shape[2] / 2)))
                        self.bin_map[i, j, k] = 1
                        print([i, j, k])
            print(i)

    def manual_map(self):
        self.bin_map[:, :, 20] = np.ones(self.map_shape[:2])
        self.bin_map[:, :, 50] = np.ones(self.map_shape[:2])
        self.bin_map[:, :, 70] = np.ones(self.map_shape[:2])

        ind = (*(np.random.rand(2) * 29).astype(int), 20)
        ind1 = (ind[0] - 15, ind[1] - 15, ind[2], 0)
        self.bin_map[ind[:3]] = 0
        ind = (*(np.random.rand(2) * 29).astype(int), 50)
        ind2 = (ind[0] - 15, ind[1] - 15, ind[2], 0)
        self.bin_map[ind[:3]] = 0
        ind = (*(np.random.rand(2) * 29).astype(int), 70)
        ind3 = (ind[0] - 15, ind[1] - 15, ind[2], 0)
        self.bin_map[ind[:3]] = 0

        big_face = np.array([(-1, -1, 0, 1), (-1, 1, 0, 1), (1, 1, 0, 1), (1, -1, 0, 1)]) / 2 * 30
        small_face = np.array([(-1, -1, 0, 1), (-1, 1, 0, 1), (1, 1, 0, 1), (1, -1, 0, 1)]) / 2
        big_face[:, 3] = 1
        small_face[:, 3] = 1
        big_face_11 = np.copy(big_face)
        big_face_11[:, 2] = -0.5
        big_face_12 = np.copy(big_face)
        big_face_12[:, 2] = 0.5
        small_face_11 = small_face + ind1
        small_face_11[:, 2] = -0.5
        small_face_12 = small_face + ind1
        small_face_12[:, 2] = 0.5
        vert_obj_1 = [*np.concatenate((big_face_11, small_face_11, big_face_12, small_face_12), axis=0)]
        faces = [[0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [0, 4, 7, 3],
                 [8, 9, 13, 12], [9, 10, 14, 13], [10, 11, 15, 14], [8, 12, 15, 11],
                 [8, 9, 1, 0], [9, 10, 2, 1], [2, 10, 11, 3], [3, 11, 8, 0],
                 [5, 13, 12, 4], [5, 13, 14, 6], [6, 14, 15, 7], [7, 15, 12, 4]]

        print(vert_obj_1)
        self.space_3d.add_object(Solid3D(self.space_3d, vert_obj_1, faces))

    def generate_map_3d_graphics(self, bin_map):
        for i in range(self.map_shape[0]):
            for j in range(self.map_shape[1]):
                for k in range(self.map_shape[2]):
                    if bin_map[i, j, k] == 1:
                        self.space_3d.add_object(Solid3D(*self.cube, (
                            i - self.map_shape[0] / 2, j - self.map_shape[1] / 2, k - self.map_shape[2] / 2)))

    def run(self):
        while self.screen.running:
            self.screen.step()

    def change_cam_mode(self, *args):
        self.space_3d.camera.mode = not self.space_3d.camera.mode
        self.space_3d.camera.reset()


trgui = TimeRrtGui(WIDTH, HEIGHT)
trgui.run()
