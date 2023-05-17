from KUKA.KUKA import YouBot
from Pygame_GUI.GUI_pygame import GuiControl
from adaptive_tools.surface_plotter import SurfaceMap
from acceleration.client import Client
import threading as thr
from path_planning.pathPlanner import PathPlanner

# Raise the KUKA server before run

ip = '192.168.88.24'

client = Client(host=ip, info=False)

robot = YouBot(ip, ros=True, offline=False, camera_enable=True, advanced=False)

surf_map = SurfaceMap(robot, client)
map_thr = thr.Thread(target=surf_map.create_surface_map, args=())
map_thr.start()

end_point = (3, 0)  # in meters
path = PathPlanner(robot)
path.run_Astar(end_point, surface=False)
weights = surf_map.weighted_map
base_point = (0, 0)
path.run_Astar(base_point, weights=weights, surface=True)

# sim = GuiControl(1200, 900, robot)
# sim.run()
