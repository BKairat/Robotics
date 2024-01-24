from camera import Camera
from robot import Robot
from pyrocon.robotCRS import robCRS93, robCRS97
import cv2
import time
import PyCapture2
import numpy as np
from task import SortA, SortB
from utils import message
import sys

def main():
    mode = sys.argv[1]
    print(len(sys.argv))
    if mode == "start":
        rob = Robot(robCRS97(), Camera(), start=True)
        message("Robot is ready!")
    elif mode == "init":
        rob = Robot(robCRS97(), Camera(), start=False)
        rob.init_pos()
        message("Robot is in init possition!")
    elif mode == "end":
        rob = Robot(robCRS97(), Camera(), start=False)
        rob.end()
        message("Robot was turned off!")
    elif mode == "reset":
        rob = Robot(robCRS97(), Camera(), start=False)
        rob.reset_motors()
        message("Motors were reseted")
    elif "sort" in mode:
        rob = Robot(robCRS97(), Camera(), start=False)
        rob.photo_pos()
        cubes, cube_ids, box, box_ids  = rob.camera.get_cubes_boxes()
        message(str(len(cubes))+" cubes and "+ str(len(box)) + " boxes were detected")
        if len(cubes) == 0:
            message("add more cubes")
        elif len(box) == 0:
            message("add more boxes")
        else: 
            if mode == "sort_a":   
                algo = SortA(rob, cubes, cube_ids, box, box_ids)
                message("Simple sorting task (successfully) started")
            elif mode == "sort_b":
                algo = SortB(rob, cubes, cube_ids, box, box_ids)
                message("Sorting with conditions (successfully) started") 
            algo.run()
            message("Sorting task complete successfully!")
    elif mode == "show":
        if len(sys.argv) == 4:
            rob = Robot(robCRS97(), Camera(), start=False)
            rob.photo_pos()
            if sys.argv[3] == "None":
                path = None
            elif sys.argv[3] == "d":
                path = "images/output_image.jpg"
            else:
                path = sys.argv[3]
            
            if int(sys.argv[2]):
                message("Show image from camera with marks (successfully) started")
                cubes, cube_ids, box, box_ids  = rob.camera.get_cubes_boxes()
                message(str(len(cubes))+" cubes and "+ str(len(box)) + " boxes were detected")
                rob.camera.show_aruco(path=path)
            else: 
                message("Show image from camera without marks (successfully) started")
                rob.camera.show_image(path=path)
            
            if path:
                message("Imagewas saved to " + path)
                    
    
if __name__ == "__main__":
    main()