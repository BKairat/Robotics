from camera import Camera
from robot import Robot
from pyrocon.robotCRS import robCRS93, robCRS97
import cv2
import time
import PyCapture2
import numpy as np
from task import Sort


rob = Robot(robCRS97(), Camera(), start=False)
rob.end()
# cubes, cube_ids, box, box_ids  = rob.camera.get_cubes_boxes()
# rob.camera.show_aruco()
# proc = Sort(rob, cubes, cube_ids, box, box_ids)
# proc.run()
