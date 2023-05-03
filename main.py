from KUKA.KUKA import YouBot
from Pygame_GUI.GUI_pygame import GuiControl
from adaptive_tools.surface_plotter import SurfaceMap
from acceleration.client import Client
import threading as thr
import time


def velocity_stream(cl, surf):
    while True:
        vel = cl.get_velocity()
        surf.velocity = vel
        print(vel)

        time.sleep(0.1)


client = Client()

robot = YouBot('192.168.88.21', ros=True, offline=True, camera_enable=True, advanced=False, ssh=False)

surf_map = SurfaceMap(robot, client)
map_thr = thr.Thread(target=surf_map.create_surface_map, args=())
map_thr.start()


vel_thr = thr.Thread(target=velocity_stream, args=(client, surf_map, ))
vel_thr.start()


sim = GuiControl(1200, 900, robot)
sim.run()

