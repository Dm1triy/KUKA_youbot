from Pygame_GUI.Screen import Screen
from Pygame_GUI.Space3D.Space3D import Space3D
from Pygame_GUI.Space3D.object_3d import *
from Pygame_GUI.Objects import *
from pathfinding.time_rrt.time_tree import Tree
import time
from Pygame_GUI.map_editor import MapEditor



class TimeRrtGui:
    def __init__(self, width, height):
        self.map_shape = (30, 30, 100)
        self.width, self.height = width, height
        self.screen = Screen(2000, 1250)
        self.screen.bg_color = (255,255,255)
        self.screen.init()

        self.map_editor = self.screen.sprite(MapEditor, "MapEditor", x=0.0, y=0, width=0.5, height=0.8,
                                             color=(255, 255, 255), map_shape=self.map_shape)
        #self.map_editor.update_map = self.export_3d

        self.screen.sprite(Button, "change_cam_mode", x=0.93, y=0.01, width=0.06, height=0.12, color=(150, 255, 170),
                           func=self.empty)
        self.screen.sprite(Button, "input_TTL", x=0.03, y=0.82, width=0.06, height=0.08, color=(150, 255, 170),
                           func=self.map_editor.input_ttl, image="sprite_images/TTL2.png")
        self.screen.sprite(Button, "export_3d", x=0.1, y=0.82, width=0.06, height=0.08, color=(150, 255, 170),
                           func=self.empty, image="sprite_images/3d.png")
        self.screen.sprite(Button, "set_origin", x=0.17, y=0.82, width=0.06, height=0.04, color=(0, 255, 0),
                           func=self.map_editor.set_mode_origin)
        self.screen.sprite(Button, "set_point", x=0.24, y=0.82, width=0.06, height=0.04, color=(0, 0, 255),
                           func=self.map_editor.set_mode_point)
        self.screen.sprite(Button, "set_wall", x=0.31, y=0.82, width=0.06, height=0.04, color=(255, 0, 255),
                           func=self.map_editor.set_mode_wall)
        self.screen.sprite(Button, "run_rrt", x=0.38, y=0.82, width=0.06, height=0.08, color=(255, 0, 255),
                           func=self.empty, image="sprite_images/pathfinding.png")
        self.screen.sprite(Button, "disable_3d", x=0.45, y=0.82, width=0.06, height=0.08, color=(255, 0, 255),
                           func=self.empty, image="sprite_images/no_3d.png")


        #self.screen.sprite(Text, "input_TTL_label", x=0.03, y=0.86, inp_text=lambda: "input_TTL", font='serif',
        #                   font_size=10)
        #self.screen.sprite(Text, "export_3d_label", x=0.1, y=0.86, inp_text=lambda: "export_3d", font='serif',
        #                   font_size=10)
        self.screen.sprite(Text, "set_origin_label", x=0.17, y=0.86, inp_text=lambda: "set_origin", font='serif',
                           font_size=10, color=(0,0,0))
        self.screen.sprite(Text, "set_point_label", x=0.24, y=0.86, inp_text=lambda: "set_point", font='serif',
                           font_size=10, color=(0,0,0))
        self.screen.sprite(Text, "set_wall_label", x=0.31, y=0.86, inp_text=lambda: "set_wall", font='serif',
                           font_size=10, color=(0,0,0))
        #self.screen.sprite(Text, "run_rrt_label", x=0.38, y=0.86, inp_text=lambda: "run_rrt", font='serif',
        #                   font_size=10)
        #self.screen.sprite(Text, "disable_3d_label", x=0.45, y=0.86, inp_text=lambda: "disable_3d", font='serif',
        #                   font_size=10)
        self.screen.sprite(Slider, "curr_time", min=0, max=self.map_shape[-1] - 1, x=0.03, y=0.91,
                           width=0.47, height=0.05, color=(150, 160, 170),
                           func=self.screen["MapEditor"].change_curr_time)

        self.old_pressed_keys = []
        self.old_mouse_pos = [0, 0]
        self.screen.add_fps_indicator()



    def empty(self, *args, **kwargs):
        print("btn pressed")
    def run(self):
        while self.screen.running:
            self.screen.step()


trgui = TimeRrtGui(WIDTH, HEIGHT)
trgui.run()
