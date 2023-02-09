import numpy as np

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

        self.space_3d = self.screen.sprite(Space3D, "Space3D", x=0.5, y=0, width=0.5, height=0.8)
        self.cube = (self.space_3d, [*np.array([(-1, -1, -1, 2), (1, -1, -1, 2), (-1, 1, -1, 2), (-1, -1, 1, 2),
                                                (1, 1, -1, 2), (1, -1, 1, 2),
                                                (-1, 1, 1, 2), (1, 1, 1, 2)]) / 2],
                     [[0, 1, 4, 2][::-1], [3, 6, 7, 5][::-1], [0, 3, 5, 1][::-1], [2, 4, 7, 6][::-1],
                      [1, 5, 7, 4][::-1], [2, 6, 3, 0][::-1]])
        self.map3d = Solid3D(*self.cube)
        self.space_3d.add_object(self.map3d)


        self.map_editor = self.screen.sprite(MapEditor, "MapEditor", x=0.0, y=0, width=0.5, height=0.8, color=(255, 255, 255), map_shape=self.map_shape)
        self.map_editor.update_map = self.export_3d

        self.screen.sprite(Button, "change_cam_mode", x=0.93, y=0.01, width=0.06, height=0.12, color=(150, 255, 170),
                           func=self.change_cam_mode)
        self.screen.sprite(Button, "input_TTL", x=0.03, y=0.82, width=0.06, height=0.04, color=(150, 255, 170),
                           func=self.map_editor.input_ttl)
        self.screen.sprite(Button, "export_3d", x=0.1, y=0.82, width=0.06, height=0.04, color=(150, 255, 170),
                           func=self.export_3d)
        self.screen.sprite(Button, "set_origin", x=0.17, y=0.82, width=0.06, height=0.04, color=(0, 255, 0),
                           func=self.map_editor.set_mode_origin)
        self.screen.sprite(Button, "set_point", x=0.24, y=0.82, width=0.06, height=0.04, color=(0, 0, 255),
                           func=self.map_editor.set_mode_point)
        self.screen.sprite(Button, "set_wall", x=0.31, y=0.82, width=0.06, height=0.04, color=(255, 0, 255),
                           func=self.map_editor.set_mode_wall)
        self.screen.sprite(Button, "run_rrt", x=0.38, y=0.82, width=0.06, height=0.04, color=(255, 0, 255),
                           func=self.run_rrt)
        self.screen.sprite(Text, "input_TTL_label", x=0.03, y=0.86, inp_text=lambda: "input_TTL", font='serif', font_size=10)
        self.screen.sprite(Text, "export_3d_label",  x=0.1, y=0.86, inp_text=lambda: "export_3d", font='serif', font_size=10)
        self.screen.sprite(Text, "set_origin_label", x=0.17, y=0.86, inp_text=lambda: "set_origin", font='serif', font_size=10)
        self.screen.sprite(Text, "set_point_label", x=0.24, y=0.86, inp_text=lambda: "set_point", font='serif', font_size=10)
        self.screen.sprite(Text, "set_wall_label", x=0.31, y=0.86, inp_text=lambda: "set_wall", font='serif', font_size=10)
        self.screen.sprite(Text, "run_rrt_label", x=0.38, y=0.86, inp_text=lambda: "run_rrt", font='serif', font_size=10)
        self.screen.sprite(Slider, "curr_time", min=0, max=self.map_shape[-1] - 1, x=0.03, y=0.91,
                           width=0.47, height=0.05, color=(150, 160, 170),
                           func=self.screen["MapEditor"].change_curr_time)

        self.old_pressed_keys = []
        self.old_mouse_pos = [0, 0]
        self.screen.add_fps_indicator()

        self.rrt_running = False

        self.bin_map = np.zeros(self.map_shape).astype(np.uint8)

    def export_3d(self, *arg, **kwargs):
        self.rrt_running = False
        self.bin_map = self.screen["MapEditor"].bin_map
        exporter_bin_map = self.screen["MapEditor"].bin_map
        for sl in range(3):
            shape = list(exporter_bin_map.shape)
            shape[sl] = 1

            zero_layer = np.zeros(shape)
            exporter_bin_map = np.append(zero_layer, exporter_bin_map, axis=sl)
            exporter_bin_map = np.append(exporter_bin_map, zero_layer, axis=sl)
        ebms = exporter_bin_map.shape
        vertices = np.array(False)
        edges = []
        curr_edge = 0
        for sl in range(3):
            for i in range(1, self.map_shape[sl]+2):
                slicer = [slice(0, ebms[0]), slice(0, ebms[1]), slice(0, ebms[2])]
                slicer[sl] = i - 1
                prev_plane = exporter_bin_map[slicer[0], slicer[1], slicer[2]]
                slicer[sl] = i
                plane = exporter_bin_map[slicer[0], slicer[1], slicer[2]]
                vertices_x = []
                vertices_y = []
                for x in range(len(plane)):
                    for y in range(len(plane[x])):
                        if prev_plane[x, y] != plane[x, y]:
                            vertices_x += [x, x, x + 1, x + 1]
                            vertices_y += [y, y + 1, y + 1, y]
                            edges += [[curr_edge, curr_edge + 1, curr_edge + 2], [curr_edge, curr_edge + 2, curr_edge + 3]]
                            curr_edge += 4
                vertices_curr = np.zeros((len(vertices_x), 4)).astype(float_bit)
                vertices_curr[:, 3] = 1

                if len(vertices_x) == 0:
                    continue
                if sl == 0:
                    vertices_curr[:, 0] = i
                    vertices_curr[:, 1] = vertices_x
                    vertices_curr[:, 2] = vertices_y

                if sl == 1:
                    vertices_curr[:, 0] = vertices_x
                    vertices_curr[:, 1] = i
                    vertices_curr[:, 2] = vertices_y
                if sl == 2:
                    vertices_curr[:, 0] = vertices_x
                    vertices_curr[:, 1] = vertices_y
                    vertices_curr[:, 2] = i
                if vertices.any():
                    vertices = np.append(vertices, vertices_curr, axis=0)
                else:
                    vertices = vertices_curr
        self.space_3d.all_obj = []
        map3d = Solid3D(self.space_3d, vertices, edges, pos=(-16.5, -16.5, -51.5))
        self.space_3d.add_object(map3d)
        self.map3d.draw_faces = True

    def run_rrt(self, *args, **kwargs):
        self.rrt = RRT(start_point=np.array([[self.map_editor.origin[0], self.map_editor.origin[1], self.map_editor.origin[2][0]]]),
                       end_point=np.array([self.map_editor.end_point[0], self.map_editor.end_point[1], self.map_editor.end_point[2][0]]), bin_map=self.bin_map)
        print(self.rrt.nodes)
        print(self.rrt.end_point)
        self.tree = Hollow3D(self.space_3d, [[0.0, 0.0, 0.0, 1.0],
                                            [0.0, 0.0, 0.0, 1.0]],
                            [[0, 0]], edges_thickness=[1])
        self.tree.translate((-15, -15, -50))
        self.space_3d.add_object(self.tree)
        #print("pathfinding took", time.time() - t, "sec")
        self.rrt.step()
        self.rrt.step()
        self.rrt.step()
        t = time.time()
        while not self.rrt.dist_reached:
            self.rrt.step()
        print(time.time()-t)
        self.rrt_running = True
    def run(self):
        while self.screen.running:
            self.screen.step()
            if self.rrt_running:
                self.rrt.step()
                self.draw_tree()

    def draw_tree(self):
        nodes = []
        for j in range(self.rrt.nodes.shape[0]):
            nodes.append(np.array([*self.rrt.nodes[j], 1]).astype(float_bit))
        edges = []
        nodes.append(np.array([*self.rrt.end_point, 1]).astype(float_bit))
        path_len = 1
        #for i in range(1, self.rrt.node_num):
        #    n = self.rrt.graph[i][0]
        #    edges.append(np.array([i, n]).astype(np.int32))
        if self.rrt.dist_reached:
            self.rrt.get_path()
            path_len = len(self.rrt.path_ind)
            edges += [np.array([self.rrt.path_ind[p], self.rrt.path_ind[p + 1]]).astype(np.int32) for p in
                      range(path_len - 1)]

        path_len -= 1
        other_edges = len(edges) - path_len
        vert_col = [[0, 255, 0], *[[255, 255, 255] for _ in range(len(nodes) - 2)], [0, 0, 255]]
        self.tree.vertex_colors = np.array(vert_col).astype(np.int32)
        vert_rad = [10, *[0] * (len(nodes) - 2), 10]
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
