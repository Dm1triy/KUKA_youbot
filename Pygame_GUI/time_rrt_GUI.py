from Pygame_GUI.Screen import Screen
from Pygame_GUI.Space3D.Space3D import Space3D
from Pygame_GUI.Space3D.object_3d import *
from Pygame_GUI.Objects import *
from pathfinding.RRT import RRT
import time
from Pygame_GUI.map_editor import MapEditor


class TimeRrtGui:
    def __init__(self, width, height):
        self.map_shape = (30, 30, 100)
        self.width, self.height = width, height
        self.screen = Screen(2000, 1250)
        self.screen.init()

        def init_3d():
            self.space_3d = self.screen.sprite(Space3D, "Space3D", x=0.5, y=0, width=0.5, height=0.8)
            self.cube = (self.space_3d, [*np.array([(-1, -1, -1, 2), (1, -1, -1, 2), (-1, 1, -1, 2), (-1, -1, 1, 2),
                                                    (1, 1, -1, 2), (1, -1, 1, 2),
                                                    (-1, 1, 1, 2), (1, 1, 1, 2)]) / 2],
                         [[0, 1, 4, 2][::-1], [3, 6, 7, 5][::-1], [0, 3, 5, 1][::-1], [2, 4, 7, 6][::-1],
                          [1, 5, 7, 4][::-1], [2, 6, 3, 0][::-1]])
            #self.manual_map()
            #self.generate_map_3d_graphics(self.bin_map)
            # self.generate_map_3d_graphics(self.bin_map)

            #self.rrt = RRT(start_point=np.array([[1, 1, 1]]), end_point=np.array([29, 29, 99]), bin_map=self.bin_map)
            #self.tree = Hollow3D(self.space_3d, [[0.0, 0.0, 0.0, 1.0],
            #                                     [0.0, 0.0, 0.0, 1.0]],
            #                     [[0, 0]], edges_thickness=[1])
            #self.tree.translate((-15, -15, -50))
            #self.space_3d.add_object(self.tree)


        init_3d()

        self.screen.sprite(MapEditor, "MapEditor", x=0.0, y=0, width=0.5, height=0.8, color=(255, 255, 255))
        self.screen.sprite(Button, "change_cam_mode", x=0.93, y=0.01, width=0.06, height=0.12, color=(150, 255, 170),
                           func=self.change_cam_mode)
        self.screen.sprite(Button, "input_TTL", x=0.03, y=0.82, width=0.06, height=0.08, color=(150, 255, 170),
                           func=self.screen["MapEditor"].input_ttl)
        self.screen.sprite(Button, "export_3d", x=0.1, y=0.82, width=0.06, height=0.08, color=(150, 255, 170),
                           func=self.export_3d)
        self.screen.sprite(Slider, "curr_time", min=0, max=self.map_shape[-1]-1, x=0.03, y=0.91,
                           width=0.47, height=0.05, color=(150, 160, 170),
                           func=self.screen["MapEditor"].change_curr_time)

        self.old_pressed_keys = []
        self.old_mouse_pos = [0, 0]
        self.screen.add_fps_indicator()

        self.bin_map = np.zeros(self.map_shape).astype(np.uint8)



    def export_3d(self, *arg, **kwargs):
        self.bin_map = self.screen["MapEditor"].bin_map
        self.generate_map_3d_graphics(self.bin_map)

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

        ind = (*(np.random.rand(2) * 9).astype(int) + 15, 20)
        ind1 = (ind[0] - 15, ind[1] - 15, ind[2] - 50, 0)
        self.bin_map[ind[0] - 5:ind[0] + 5, ind[1] - 5:ind[1] + 5, ind[2]] = 0
        ind = (*(np.random.rand(2) * 9).astype(int) + 15, 50)
        ind2 = (ind[0] - 15, ind[1] - 15, ind[2] - 50, 0)
        self.bin_map[ind[0] - 5:ind[0] + 5, ind[1] - 5:ind[1] + 5, ind[2]] = 0
        ind = (*(np.random.rand(2) * 9).astype(int) + 15, 70)
        ind3 = (ind[0] - 15, ind[1] - 15, ind[2] - 50, 0)
        self.bin_map[ind[0] - 5:ind[0] + 5, ind[1] - 5:ind[1] + 5, ind[2]] = 0

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

            # self.space_3d.add_object(Solid3D(self.space_3d, vert_obj_1, faces))

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
        print_time = True
        t = time.time()
        self.rrt.step()
        self.rrt.step()
        self.rrt.step()
        while self.screen.running:
            self.screen.step()
            self.rrt.step()
            if self.rrt.dist_reached:
                if print_time:
                    print("pathfinding took", time.time() - t, "sec")
                    print_time = False
                self.screen.step()
                #self.draw_tree()

    def draw_tree(self):
        nodes = []
        for j in range(self.rrt.nodes.shape[0]):
            nodes.append(np.array([*self.rrt.nodes[j], 1]).astype(float_bit))
        edges = []
        nodes.append(np.array([*self.rrt.end_point, 1]).astype(float_bit))
        path_len = 1
        for i in range(1, self.rrt.node_num):
            n = self.rrt.graph[i][0]
            edges.append(np.array([i, n]).astype(np.int32))
        if self.rrt.dist_reached:
            self.rrt.get_path()
            path_len = len(self.rrt.path_ind)
            edges += [np.array([self.rrt.path_ind[p], self.rrt.path_ind[p + 1]]).astype(np.int32) for p in
                      range(path_len - 1)]

        path_len -= 1
        other_edges = len(edges) - path_len
        vert_col = [[0, 0, 255],  *[[255, 255, 255] for _ in range(len(nodes)-2)], [0, 255, 0]]
        self.tree.vertex_colors = np.array(vert_col).astype(np.int32)
        vert_rad = [10, *[1] * (len(nodes)-2), 10]
        self.tree.vertex_radius = np.array(vert_rad).astype(np.int16)
        self.tree.vertices = np.array(nodes)

        self.tree.edges_thickness = np.array([*[1] * other_edges, *[5] * path_len]).astype(np.int16)
        self.tree.color_edges = np.array(
            [*[[255, 255, 255] for _ in range(other_edges)], *[[255, 0, 0] for _ in range(path_len)]]).astype(np.int32)
        self.tree.edges = np.array(edges).astype(np.int32)

    def change_cam_mode(self, *args, **kwargs):
        self.space_3d.camera.mode = not self.space_3d.camera.mode
        self.space_3d.camera.reset()


trgui = TimeRrtGui(WIDTH, HEIGHT)
trgui.run()
