from KUKA.KUKA import KUKA
from Pygame_GUI.GUI_pygame import GuiControl

robot = KUKA('192.168.88.21', ros=False, offline=False, advanced=False)
# robot = KUKA('192.168.88.24', ros=True, offline=False)
#robot = ['192.168.88.21', '192.168.88.22', '192.168.88.23', '192.168.88.24', '192.168.88.25']

# new_map = MapPlotter(robot)
# map_thr = thr.Thread(target=new_map.create_map, args=())
# map_thr.start()

sim = GuiControl(3000, 2000, robot)
sim.run()
