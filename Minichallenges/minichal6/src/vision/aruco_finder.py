#!/usr/bin/env python3 

import rospy 
import numpy as np
from fiducial_msgs.msg import FiducialTransformArray

class ArucoFinder:
    def __init__(self):
        ###--- Inicio del Nodo ---###
        rospy.init_node('aruco_finder')
        rospy.on_shutdown(self.cleanup)

        ###--- Subscriptores ---###
        rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.ft_cb)
    
        ###--- Robot Constants ---###
        self.dt = 0.02
        self.id = 703
        self.camera_t_robot = np.array([
            [1.0, 0.0, 0.0, 0.06],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.065],
            [0.0, 0.0, 0.0, 1.0]
        ])

        ###--- Variables ---####
        self.fiducial_transform = FiducialTransformArray()
        rate = rospy.Rate(int(1.0 / self.dt))

        while rospy.get_time() == 0:
            pass # Esperar a que el tiempo de ROS esté inicializado 

        print("Nodo operando")

        while not rospy.is_shutdown():
            self.process_transforms()
            rate.sleep()

    def ft_cb(self, msg):
        self.fiducial_transform = msg

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

                rospy.loginfo("Distance to marker: %f", distance)
                rospy.loginfo("Angle to marker: %f degrees", angle_deg)

    def transform_marker_position(self, cam_p_aruco):
        # Transform to robot frame
        cam_p_aruco_homogeneous = np.append(cam_p_aruco, 1.0)  # Convert to homogeneous coordinates
        robot_p_aruco_homogeneous = np.dot(self.camera_t_robot, cam_p_aruco_homogeneous)
        robot_p_aruco = robot_p_aruco_homogeneous[:3]  # Convert back to Cartesian coordinates
        return robot_p_aruco

    def cleanup(self):
        print("Apagando Localización")

if __name__ == "__main__":
    ArucoFinder()
