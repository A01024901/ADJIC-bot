#!/usr/bin/env python3 

import rospy 
import numpy as np
from fiducial_msgs.msg import FiducialTransformArray
import tf.transformations as tft

class aruco_finder:
    def __init__(self):
        ###--- Inicio del Nodo ---###
        rospy.init_node('aruco_finder')
        rospy.on_shutdown(self.cleanup)

        ###--- Subscriptores ---###
        rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.ft_cb)
    
        ###--- Publishers ---###
        # self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=1)

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
        
        ###--- Objects ---###
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
                print("Pose: ", aruco.transform.translation)
                print("Orientation: ", aruco.transform.rotation)
                print(" ")

                cam_p_aruco = np.array([
                    aruco.transform.translation.x, 
                    aruco.transform.translation.y, 
                    aruco.transform.translation.z
                ])
                cam_q_aruco = np.array([
                    aruco.transform.rotation.x, 
                    aruco.transform.rotation.y, 
                    aruco.transform.rotation.z, 
                    aruco.transform.rotation.w
                ])

                robot_p_aruco = self.transform_marker_position(cam_p_aruco, cam_q_aruco)
                rospy.loginfo("Position in robot frame: %s", robot_p_aruco)

                distance = np.linalg.norm(robot_p_aruco)
                angle = np.arctan2(robot_p_aruco[1], robot_p_aruco[0])

                rospy.loginfo("Distance to marker: %f", distance)
                rospy.loginfo("Angle to marker: %f", angle)

    def transform_marker_position(self, cam_p_aruco, cam_q_aruco):
        # Convert quaternion to rotation matrix
        cam_R_aruco = tft.quaternion_matrix(cam_q_aruco)

        # Create homogeneous transformation matrix for the marker in camera frame
        cam_T_aruco = np.identity(4)
        cam_T_aruco[:3, :3] = cam_R_aruco[:3, :3]
        cam_T_aruco[:3, 3] = cam_p_aruco

        # Transform to robot frame
        robot_T_aruco = np.dot(self.camera_t_robot, cam_T_aruco)
        robot_p_aruco = robot_T_aruco[:3, 3]
        return robot_p_aruco

    def cleanup(self):
        print("Apagando Localización")

if __name__ == "__main__":
    aruco_finder()
