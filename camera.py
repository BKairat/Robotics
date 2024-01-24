
import cv2
import time
import PyCapture2
import numpy as np
import math
import matplotlib.pyplot as plt 
from task import Object


def rotationMatrixToEulerAngles(R):  
    sy = np.sqrt(R[0,0] * R[0,0] + R[1,0] * R[1,0])

    singular = sy < 1e-6

    if not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else:
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
    return np.array([x, y, z]) * (180/np.pi) 


class Camera():
    def __init__(self):
        self.bus = PyCapture2.BusManager()
        self.camera = PyCapture2.Camera()
        self.camera.connect(self.bus.getCameraFromIndex(0))
        self.camera.startCapture()

    def get_image(self):
        image = self.camera.retrieveBuffer()
        image = image.convert(PyCapture2.PIXEL_FORMAT.RGB8)
        rgb_cv_image = np.array(image.getData(), dtype="uint8").reshape((image.getRows(), image.getCols(), 3));
        return cv2.cvtColor(rgb_cv_image, cv2.COLOR_RGB2BGR)
    
    def show_image(self, path=None):
        img = self.get_image()
        img_ = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        plt.imshow(img_)
        plt.show()
        if path:
            cv2.imwrite(path, img)

    @staticmethod    
    def scale(polygon, scale_factor):
        center = np.mean(polygon, axis=0)
        translated_polygon = polygon - center
        scaled_polygon = translated_polygon * scale_factor
        final_polygon = scaled_polygon + center
        return final_polygon

        
    def show_aruco(self, path=None):
        image = self.get_image()
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_1000)

        parameters = cv2.aruco.DetectorParameters_create()
        
        parameters.adaptiveThreshConstant = 30
        (corners, ids, _) = cv2.aruco.detectMarkers(gray, aruco_dict, parameters = parameters)
        cv2.aruco.drawDetectedMarkers(image, corners, ids)

        plt.imshow(image)
        plt.show()
        if path:
            cv2.imwrite(path, image)
    
    
    def get_cubes_boxes(self):
        '''
        Compute transformation from camera to target(aruco marker)
        Return:
            list of aruco markers ids and theirs tvecs and rotation angle for gripper roll
        '''
        image = self.get_image()

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_1000)
        parameters = cv2.aruco.DetectorParameters_create()
        parameters.adaptiveThreshConstant = 30
        (corners, ids, _) = cv2.aruco.detectMarkers(gray, aruco_dict, parameters = parameters)

        camera_matrix = np.array(
            [
                [4.52084817e+03, 0.00000000e+00, 4.65318015e+02],
                [0.00000000e+00, 4.52084817e+03, 2.97909076e+02],
                [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
            ]
        )
        distortion = np.zeros(5)
        
        cubes_list = []
        cubes_ids = []
        boxes_list = []
        boxes_ids = []
        for i in range(len(ids)):
            '''
            tvec = [x, y, z]
            rvec = [yaw, pitch, roll]
            '''
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners[i], 0.04, camera_matrix, distCoeffs=distortion
            )
            corners_shifted = np.concatenate([corners[i][:, 2:, :], corners[i][:, :2, :]], axis=1)
            _, tvec_shifted, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners_shifted, 0.04, camera_matrix, distCoeffs=distortion
            )
            
            rotation_matrix, _ = cv2.Rodrigues(rvec)
            roll = rotationMatrixToEulerAngles(rotation_matrix)
            x1, y1, z1 = tvec[0][0]  
            x2, y2, z2 = tvec_shifted[0][0]
            x, y, z = (x1+x2)/2, (y1+y2)/2, (z1+z2)/2

            if int(ids[i][0]) < 4: #ids is 0,1,2,3
                cubes_list.append(Object(id=int(ids[i][0]),
                                         conf=np.array([x,y,z,roll[2]]),
                                         corn=self.scale(corners[i].reshape((4,2)), 1.5).reshape((4,2))))   
                if int(ids[i][0]) not in cubes_ids:
                    cubes_ids.append(int(ids[i][0]))
            else:
                boxes_list.append(Object(id=int(ids[i][0]),
                                         conf=np.array([x,y,z,roll[2]]),
                                         corn=corners[i].reshape(4,2)))  
                if int(ids[i][0]) not in boxes_ids:
                    boxes_ids.append(int(ids[i][0]))
        return cubes_list, cubes_ids, boxes_list, boxes_ids

    def homography(self, tvec_C):
        H = np.array([[ -6.96742407e+02,   9.92215832e+00,   5.43884477e+02],
                    [  2.68609387e+00,   6.98686793e+02,  -1.76555556e+02],
                    [  2.90336258e-02,   1.74486268e-02,   1.00000000e+00]])
        tvec_FK = np.matmul(H,np.append(tvec_C[:2],[1]))
        return tvec_FK