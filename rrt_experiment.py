from Pygame_GUI.Screen import Screen
from Pygame_GUI.Space3D.Space3D import Space3D
from Pygame_GUI.Space3D.object_3d import *
from Pygame_GUI.Objects import *
from pathfinding.time_rrt.time_tree import Tree
import time
from Pygame_GUI.map_editor_2d import MapEditor2d



class RrtTestGui:
    def __init__(self, width, height):
        self.map_shape = (800, 800)
        self.width, self.height = width, height
        self.screen = Screen(1000, 1250)
        self.screen.init()

        self.map_editor = self.screen.sprite(MapEditor2d, "MapEditor", x=0.0, y=0, width=1, height=0.8,
                                             color=(255, 255, 255), map_shape=self.map_shape)
        self.screen.sprite(Button, "set_origin", x=0.24, y=0.82, width=0.12, height=0.04, color=(0, 255, 0),
                           func=self.map_editor.set_mode_origin)
        self.screen.sprite(Button, "set_point", x=0.38, y=0.82, width=0.12, height=0.04, color=(0, 0, 255),
                           func=self.map_editor.set_mode_point)
        self.screen.sprite(Button, "set_wall", x=0.52, y=0.82, width=0.12, height=0.04, color=(132, 31, 39),
                           func=self.map_editor.set_mode_wall)
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

        self.tree = Tree(start_point=np.array(start_point), end_point=np.array(end_point), bin_map=self.bin_map)
        self.tree.step()
        self.tree_running = True


    def run(self):
        while self.screen.running:
            self.screen.step()




trgui = RrtTestGui(WIDTH, HEIGHT)
trgui.run()
