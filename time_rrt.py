from Pygame_GUI.Screen import Screen
from Pygame_GUI.Space3D.Space3D import Space3D
from Pygame_GUI.Space3D.object_3d import *
from Pygame_GUI.Objects import *
from Pygame_GUI.constants import *
from pathfinding.RRT import RRT


class TimeRrtGui:
    def __init__(self, width, height):
        self.map_shape = (30, 30, 100)
        self.width, self.height = width, height
        self.screen = Screen(1000, 1000)
        self.space_3d = Space3D(self.screen, x=0, y=0, width=1, height=1)
        self.old_pressed_keys = []
        self.old_mouse_pos = [0, 0]
        self.fps_text = Text(self.screen, x=0.1, y=0.01, inp_text=self.screen.get_fps, font='serif', font_size=10)
        # self.space_3d.load_object_from_file("Pygame_GUI/Space3D/teapot.obj", (5, 0, 0))
        self.cube = (self.space_3d, [*np.array([(-1, -1, -1, 2), (1, -1, -1, 2), (-1, 1, -1, 2), (-1, -1, 1, 2),
                                                (1, 1, -1, 2), (1, -1, 1, 2),
                                                (-1, 1, 1, 2), (1, 1, 1, 2)]) / 2],
                     [[0, 1, 4, 2][::-1], [3, 6, 7, 5][::-1], [0, 3, 5, 1][::-1], [2, 4, 7, 6][::-1],
                      [1, 5, 7, 4][::-1], [2, 6, 3, 0][::-1]])
        #self.space_3d.add_object(Axes(self.space_3d))
        Button(self.screen, x=0.9, y=0.01, width=0.08, height=0.06, color=(150, 255, 170), func=self.change_cam_mode)
        self.bin_map = np.zeros(self.map_shape).astype(np.uint8)
        # self.generate_random_map()
        self.manual_map()
        self.generate_map_3d_graphics(self.bin_map)
        # self.generate_map_3d_graphics(self.bin_map)

        self.rrt = RRT(start_point=np.array([[1, 1, 1]]), end_point=np.array([30, 30, 100]), bin_map=self.bin_map)
        self.tree = Hollow3D(self.space_3d, [[0.0, 0.0, 0.0, 1.0],
                                             [0.0, 0.0, 0.0, 1.0]],
                             [[0, 0]], edges_thickness=[1])
        self.tree.translate((-15, -15, -50))
        self.space_3d.add_object(self.tree)

    def generate_random_map(self):
        rand_arr = np.random.rand(*self.map_shape)
        for i in range(self.map_shape[0]):
            for j in range(self.map_shape[1]):
                for k in range(self.map_shape[2]):
                    if rand_arr[i, j, k] > 0.999:
                        self.space_3d.add_object(Solid3D(*self.cube, (
                            i - self.map_shape[0] / 2, j - self.map_shape[1] / 2, k - self.map_shape[2] / 2)))
                        self.bin_map[i, j, k] = 1

    def manual_map(self):
        self.bin_map[:, :, 20] = np.ones(self.map_shape[:2])
        self.bin_map[:, :, 50] = np.ones(self.map_shape[:2])
        self.bin_map[:, :, 70] = np.ones(self.map_shape[:2])

        ind = (*(np.random.rand(2) * 9).astype(int)+15, 20)
        ind1 = (ind[0] - 15, ind[1] - 15, ind[2] - 50, 0)
        self.bin_map[ind[0]-5:ind[0]+5, ind[1]-5:ind[1]+5, ind[2]] = 0
        ind = (*(np.random.rand(2) * 9).astype(int)+15, 50)
        ind2 = (ind[0] - 15, ind[1] - 15, ind[2] - 50, 0)
        self.bin_map[ind[0]-5:ind[0]+5, ind[1]-5:ind[1]+5, ind[2]] = 0
        ind = (*(np.random.rand(2) * 9).astype(int)+15, 70)
        ind3 = (ind[0] - 15, ind[1] - 15, ind[2] - 50, 0)
        self.bin_map[ind[0]-5:ind[0]+5, ind[1]-5:ind[1]+5, ind[2]] = 0

        big_face = np.array([(-1, -1, 0.0, 1), (-1, 1, 0.0, 1), (1, 1, 0.0, 1), (1, -1, 0.0, 1)]) / 2 * 30
        small_face = np.array([(-1, -1, 0.0, 1), (-1, 1, 0.0, 1), (1, 1, 0.0, 1), (1, -1, 0.0, 1)]) * 5
        big_face[:, 3] = 1
        small_face[:, 3] = 1
        add_obj_array = [ind1, ind2, ind3]
        for param in add_obj_array:
            big_face_11 = np.copy(big_face)
            big_face_11[:, 2] = -0.5 + param[2]
            big_face_12 = np.copy(big_face)
            big_face_12[:, 2] = 0.5 + param[2]
            small_face_11 = small_face + param
            small_face_11[:, 2] -= 0.5
            small_face_12 = small_face + param
            small_face_12[:, 2] += 0.5
            vert_obj_1 = [*np.concatenate((big_face_11, small_face_11, big_face_12, small_face_12), axis=0)]
            faces = [[0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [0, 4, 7, 3],
                     [8, 9, 13, 12], [9, 10, 14, 13], [10, 11, 15, 14], [8, 12, 15, 11],
                     [8, 9, 1, 0], [9, 10, 2, 1], [2, 10, 11, 3], [3, 11, 8, 0],
                     [5, 13, 12, 4], [5, 13, 14, 6], [6, 14, 15, 7], [7, 15, 12, 4]]

            #self.space_3d.add_object(Solid3D(self.space_3d, vert_obj_1, faces))

    def generate_map_3d_graphics(self, bin_map):
        for i in range(self.map_shape[0]):
            for j in range(self.map_shape[1]):
                for k in range(self.map_shape[2]):
                    if bin_map[i, j, k] == 1:
                        self.space_3d.add_object(Solid3D(*self.cube, (
                            i - self.map_shape[0] / 2, j - self.map_shape[1] / 2, k - self.map_shape[2] / 2)))

    def run(self):
        self.rrt.step()
        self.rrt.step()
        self.rrt.step()
        while self.screen.running:
            self.screen.step()
            self.rrt.step()
            self.draw_tree()


    def draw_tree(self):
        nodes = []
        for j in range(self.rrt.nodes.shape[0]):
            nodes.append(np.array([*self.rrt.nodes[j], 1]).astype(float_bit))
        edges = []
        for i in range(1, self.rrt.node_num):
            n = self.rrt.graph[i][0]
            edges.append(np.array([i, n]).astype(np.uint16))
        self.tree.edges_thickness = np.array([1] * len(edges)).astype(np.int16)
        self.tree.vertex_colors = np.array([[255, 255, 255] for _ in nodes]).astype(np.uint32)
        self.tree.vertex_radius = np.array([2] * len(nodes)).astype(np.int16)
        self.tree.color_edges = np.array([[255, 255, 255] for _ in edges]).astype(np.uint32)
        self.tree.vertices = np.array(nodes)
        self.tree.edges = np.array(edges).astype(np.uint32)

    def change_cam_mode(self, *args, **kwargs):
        self.space_3d.camera.mode = not self.space_3d.camera.mode
        self.space_3d.camera.reset()


trgui = TimeRrtGui(WIDTH, HEIGHT)
trgui.run()
