import argparse
import numpy as np
from pyrocon.CRS_commander import Commander

from pyrocon.graph import Graph
from pyrocon.interpolation import *
from pyrocon.robCRSgripper import robCRSgripper
from pyrocon.robotCRS import robCRS93, robCRS97
from pyrocon.robCRSdkt import robCRSdkt

import cv2

from utils import message
# from shapely.geometry import Polygon
from camera import Camera


class Robot():
    def __init__(self, rob, camera, tty_dev = "/dev/ttyUSB0", start = True):
        self.com = Commander(rob)
        self.com.open_comm(tty_dev, speed=19200)
        self.camera = camera
        if start:
            self.com.init()


    def move_to(self, pos, height, pitch, roll):
        pos = np.array([pos[0],pos[1],pos[2], 0, pitch, roll])
        if pos[0] > 550:
            pos += np.array([15, 0, 0, 0, 0, 0])
        else:
            pos += np.array([10, 0, 0, 0, 0, 0])
        pos[2] = height
        pos[4] = pitch
        pos[5] = roll
        while True:
            try:
                p_irc = self.com.find_closest_ikt(pos) 
                break
            except ValueError:
                pos[4] -= 1
                if pos[4] < 50:
                    message('Unreachable position') 
                    return False
        p_irc = self.com.find_closest_ikt(pos)
        self.com.coordmv(p_irc)
        self.com.wait_ready(sync=True)
        return pos
    
    def move_to_box(self, box):
        roll = box.obj[3]
        pitch = 90
        tvec_C = box.obj[:3]
        tvec_FK = self.camera.homography(np.append(tvec_C[:2], [1]))
        self.move_to(tvec_FK, 200, pitch, roll)
        self.release()
        
        
    def grab_cube(self, cube):
        roll = cube.obj[3]
        pitch = 90
        tvec_C = cube.obj[:3]
        tvec_FK = self.camera.homography(np.append(tvec_C[:2], [1]))
        self.move_to(tvec_FK, 150, pitch, roll)
        self.move_to(tvec_FK, 45, pitch, roll)
        self.grab()
    
    def grab(self):
        robCRSgripper(self.com, 0.9)
        self.com.wait_ready()

    def release(self):
        robCRSgripper(self.com, -10)
        self.com.wait_ready()
        
    def init_pos(self):
        self.com.soft_home()

    def photo_pos(self):
        self.com.soft_home()
        irc = self.com.axis_get_pos()[1]
        irc[2] = -40000
        self.com.move_to_pos(irc)
        self.com.wait_ready()
    
    def end(self):
        self.com.soft_home()
        self.com.rcon.close()
        message("please turn off the robot")
    
