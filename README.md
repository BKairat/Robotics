# Robotics
___
This project created for aruco cubes sorting task by CRS robot.
+ **Task:**  We have some cubes with aruco marks on the ground and we want to sort them by its ids into boxes which has also aruco marks.
+ **Robot:**  CRS97
+ **Camera:**  [Chameleon](https://www.flir.com/iis/machine-vision/) manufactured by PointGrey.
+ **Requirements:** 
  + Python 2.7
  + cv2
  + numpy
  + matplotlib
  + [Pyrocon](https://github.com/cvut/pyrocon) (already here)
---
Example of usage:
``python main.py`` + ``mode``
modes:
``sort_a`` in this mode will run the robot for sorting cubes that are not touching.
``sort_b`` in this mode will run the robot for sorting cubes that can touch.
``start`` prepare robot to use it.
``init`` set robot in init configuration.
``reset`` reset its motors.
``end`` prepare robot before it would be switched off.
``show`` show image from camera, requires other argeuments: "1" to detect aruco amrkers or "0" to not, path where the imag will be saved ``d`` default path ``images/output_image.jpg``, ``None`` to not save or you can write yourth one.

<br>
<p align="center">
  <img src="IMG_7818.gif" />
</p>