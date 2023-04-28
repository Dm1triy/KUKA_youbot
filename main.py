from KUKA.KUKA import YouBot
from Pygame_GUI.GUI_pygame import GuiControl
from adaptive_tools.surface_plotter import SurfaceMap
import threading as thr


robot = YouBot('192.168.88.21', ros=True, offline=False, camera_enable=True, advanced=False, ssh=False)

# surf_map = SurfaceMap(robot)
# map_thr = thr.Thread(target=surf_map.create_surface_map, args=())
# map_thr.start()

sim = GuiControl(1200, 900, robot)
sim.run()

