from KUKA.KUKA import KUKA
from Pygame_GUI.GUI_pygame import GuiControl
from adaptive_tools.surface_plotter import SurfaceMap
import threading as thr

robot = KUKA('192.168.88.21', ros=False, offline=False, camera_enable=True, advanced=False)
#robot = ['192.168.88.21', '192.168.88.22', '192.168.88.23', '192.168.88.24', '192.168.88.25']

surf_map = SurfaceMap(robot)
map_thr = thr.Thread(target=surf_map.create_surface_map, args=())

map_thr.start()

sim = GuiControl(1200, 900, robot)
sim.run()
