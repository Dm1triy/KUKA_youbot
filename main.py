from KUKA.KUKA import KUKA
from Pygame_GUI.GUI_pygame import GuiControl

robot = KUKA('192.168.88.21', ros=True, offline=False, camera_enable=True, advanced=False, log=("new/example3.txt", 5))
#robot = ['192.168.88.21', '192.168.88.22', '192.168.88.23', '192.168.88.24', '192.168.88.25']
sim = GuiControl(1200, 900, robot)
sim.run()
