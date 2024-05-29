#!/usr/bin/env python3 

import rospy 
import numpy as np
from fiducial_msgs.msg import FiducialTransformArray

class aruco:
    def __init__(self , ID , x , y):
        self.ID = ID
        self.x = x
        self.y = y

        self.camera_t_robot = np.array([
            [0.0, 0.0, 1.0, 0.1],
            [1.0, 0.0, 0.0, 0.0],
            [0.0, -1.0, 0.0, 0.065],
            [0.0, 0.0, 0.0, 1.0]])
        
        self.aruco2origin = np.array([
            [1, 0.0, 0, x],
            [0, 1, 0.0, y],
            [0.0, 0, 1, 0.1],
            [0.0, 0.0, 0.0, 1.0]])
        
    def get_trasnform():
        pass


class ArucoFinder:
    def __init__(self):
        ###--- Inicio del Nodo ---###
        rospy.init_node('aruco_finder')
        rospy.on_shutdown(self.cleanup)

        ###--- Subscriptores ---###
        rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.ft_cb)
    
        ###--- Constants ---###
        self.dt = 0.02
        self.robotI2origin = np.array([
            [1, 0.0, 0.0, 0.1],
            [0.0, 1, 0.0, 1.7],
            [0.0, 0.0, 1, 0],
            [0.0, 0.0, 0.0, 1.0]])

        ###--- Objetos ---####
        self.arucos = [aruco(702 , 0.0 , 0.80) , aruco(701 , 0.0 , 1.60) , aruco(703 , 1.73 , 0.80) , 
                       aruco(704 , 2.63 , 0.39) , aruco(705 , 2.85 , 0.0) , aruco(706 , 2.87 , 1.22)]
        
        self.arucos_sim = [aruco(702 , 0.0 , 0.80) , aruco(701 , 0.0 , 1.60) , aruco(703 , 1.73 , 0.80) , 
                       aruco(704 , 2.63 , 0.39) , aruco(705 , 2.85 , 0.0) , aruco(706 , 2.87 , 1.22)]
        
        self.fiducial_transform = FiducialTransformArray()
        rate = rospy.Rate(int(1.0 / self.dt))

        while rospy.get_time() == 0: pass 

        while not rospy.is_shutdown():
            self.process_transforms()
            rate.sleep()

    def process_transforms(self):
        for aruco in self.fiducial_transform.transforms:
            if aruco.fiducial_id == self.id:
                print("Fiducial ID: ", aruco.fiducial_id)

                cam_p_aruco = np.array([
                    aruco.transform.translation.x, 
                    aruco.transform.translation.y, 
                    aruco.transform.translation.z
                ])

                robot_p_aruco = self.transform_marker_position(cam_p_aruco)

                distance = np.linalg.norm(robot_p_aruco)
                angle_rad = np.arctan2(robot_p_aruco[1], robot_p_aruco[0])
                angle_deg = np.degrees(angle_rad)

                print("Distance to marker: %f", distance)
                print("Angle to marker: %f degrees", angle_deg)

    def transform_marker_position(self, cam_p_aruco):
        # Transform to robot frame
        cam_p_aruco_homogeneous = np.append(cam_p_aruco, 1.0)  # Convert to homogeneous coordinates
        robot_p_aruco_homogeneous = np.dot(self.camera_t_robot, cam_p_aruco_homogeneous)
        robot_p_aruco = robot_p_aruco_homogeneous[:3]  # Convert back to Cartesian coordinates
        return robot_p_aruco
    
    def ft_cb(self, msg):
        self.fiducial_transform = msg

    def cleanup(self):
        print("Apagando Localización")

if __name__ == "__main__":
    ArucoFinder()