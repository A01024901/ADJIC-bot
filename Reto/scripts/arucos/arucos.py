#!/usr/bin/env python3

import rospy 
import numpy as np
from fiducial_msgs.msg import FiducialTransformArray
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray

class aruco:
    def __init__(self , ID , x , y):
        self.ID = ID
        self.x = x
        self.y = y

        self.robot2camera = np.array([
            [0.0, 0.0, 1.0, 0.1],
            [1.0, 0.0, 0.0, 0.0],
            [0.0, -1.0, 0.0, 0.065],
            [0.0, 0.0, 0.0, 1.0]])
        
        self.origin2aruco = np.array([
            [1, 0.0, 0, x],
            [0, 1, 0.0, y],
            [0.0, 0, 1, 0.1],
            [0.0, 0.0, 0.0, 1.0]])
        
    def transform_origin2robot(self , x , y  , z):
        camera2aruco = np.array([
            [1, 0.0, 0, x],
            [0, 1, 0.0, y],
            [0.0, 0, 1, z],
            [0.0, 0.0, 0.0, 1.0]])
        
        aruco2camera = self.get_inv(camera2aruco)
        camera2robot = self.get_inv(self.robot2camera)
        
        self.origin2robot = self.origin2aruco.dot(aruco2camera).dot(camera2robot)

        return self.origin2robot
    
    def transform_robot2aruco(self , x , y  , z):
        camera2aruco = np.array([
            [1, 0.0, 0, x],
            [0, 1, 0.0, y],
            [0.0, 0, 1, z],
            [0.0, 0.0, 0.0, 1.0]])
        
        robot2aruco = self.robot2camera.dot(camera2aruco)

        return robot2aruco

    def get_inv(self , T):
        R = T[:3, :3]
        t = T[:3, 3]
        
        R_inv = R.T
        
        t_inv = -R_inv @ t
        
        T_inv = np.eye(4)
        T_inv[:3, :3] = R_inv
        T_inv[:3, 3] = t_inv
        
        return T_inv
    

