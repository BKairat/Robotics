
import cv2
import time
import PyCapture2
import numpy as np
import math
import matplotlib.pyplot as plt 


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
    
    def show_image(self):
        img = self.get_image()
        plt.imshow(img)
        plt.show()
    
    @staticmethod    
    def scale(corners, factor):
        ret = []
        scale=np.array([[factor, 0], [0, factor]])
        for polygon in corners:
            center = np.mean(polygon, axis=0)
            polygon = polygon - center
            polygon = np.matmul(polygon, scale) 
            ret.append(np.array(polygon))
        return tuple(ret)
        
    def show_aruco(self):
        image = self.get_image()
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_1000)

        parameters = cv2.aruco.DetectorParameters_create()
        
        parameters.adaptiveThreshConstant = 30
        (corners, ids, _) = cv2.aruco.detectMarkers(gray, aruco_dict, parameters = parameters)
        print("corners",corners)
        
        new = self.scale(corners, 1.72)
        print("NEW",new)

        cv2.aruco.drawDetectedMarkers(image, corners, ids)
        cv2.aruco.drawDetectedMarkers(image, new)

        plt.imshow(image)
        plt.show()

    
    def get_cubes_boxes(self):
        '''
        Compute transformation from camera to target(aruco marker)
        Arg:
            image: name of image from where we get camera translation
        Return:
            list of aruco markers ids and theirs tvecs and rotation angle for gripper roll
        '''
        image = self.get_image()

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_1000)
        parameters = cv2.aruco.DetectorParameters_create()
        parameters.adaptiveThreshConstant = 30
        (corners, ids, rejected) = cv2.aruco.detectMarkers(gray, aruco_dict, parameters = parameters)

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

            rotation_matrix, _ = cv2.Rodrigues(rvec)
            roll = rotationMatrixToEulerAngles(rotation_matrix)
            x, y, z = tvec[0][0]  

            if int(ids[i][0]) < 4: #ids is 0,1,2,3
                cubes_list.append(np.array([int(ids[i][0]),x,y,z,roll[2]]))   
                if int(ids[i][0]) not in cubes_ids:
                    cubes_ids.append(int(ids[i][0]))
            else:
                boxes_list.append(np.array([int(ids[i][0]),x,y,z,roll[2]]))
                if int(ids[i][0]) not in boxes_ids:
                    boxes_ids.append(int(ids[i][0]))
        return cubes_list, cubes_ids, boxes_list, boxes_ids

    # @staticmethod
    def homography(self, tvec_C):
        H = np.array([[ -6.96742407e+02,   9.92215832e+00,   5.43884477e+02],
                    [  2.68609387e+00,   6.98686793e+02,  -1.76555556e+02],
                    [  2.90336258e-02,   1.74486268e-02,   1.00000000e+00]])
        tvec_FK = np.matmul(H,np.append(tvec_C[:2],[1]))
        return tvec_FK