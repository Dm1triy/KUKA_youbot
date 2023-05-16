from KUKA.KUKA import YouBot
from Pygame_GUI.GUI_pygame import GuiControl
from adaptive_tools.surface_plotter import SurfaceMap
from acceleration.client import Client
import threading as thr

# Raise the KUKA server before run

ip = '192.168.88.24'

client = Client(host=ip, info=False)

robot = YouBot(ip, ros=False, offline=False, camera_enable=True, advanced=False)

surf_map = SurfaceMap(robot, client)
map_thr = thr.Thread(target=surf_map.create_surface_map, args=())
map_thr.start()


sim = GuiControl(1200, 900, robot)
sim.run()
