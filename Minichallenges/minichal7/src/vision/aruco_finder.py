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
    
class ArucoFinder:
    def __init__(self , mode):
        ###--- Inicio del Nodo ---###
        rospy.init_node('aruco_finder')
        rospy.on_shutdown(self.cleanup)

        ###--- Subscriptores ---###
        rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.ft_cb)

        ###--- Publishers ---###
        self.array_pub = rospy.Publisher("/ar_array" , Float32MultiArray , queue_size=1)
        self.flag_pub = rospy.Publisher("/arucos_flag" , Bool , queue_size=1)
    
        ###--- Constants ---###
        self.dt = 0.02
        self.robotI2origin = np.array([
            [1, 0.0, 0.0, 0.1],
            [0.0, 1, 0.0, 1.7],
            [0.0, 0.0, 1, 0],
            [0.0, 0.0, 0.0, 1.0]])

        ###--- Objetos ---####
        arucos = [aruco(702 , 0.0 , 0.80) , aruco(701 , 0.0 , 1.60) , aruco(703 , 1.73 , 0.80) , 
                  aruco(704 , 2.63 , 0.39) , aruco(705 , 2.85 , 0.0) , aruco(706 , 2.87 , 1.22) ,
                  aruco(707 , 1.74 , 1.22)]
        
        arucos_sim = [aruco(0 , 0.0 , 0.80) , aruco(1 , 0.0 , 1.60) , aruco(2 , 1.73 , 0.80) , 
                       aruco(3 , 2.63 , 0.39) , aruco(4 , 2.85 , 0.0) , aruco(5 , 2.87 , 1.22)]
        
        if mode == "sim": self.arucos = arucos_sim
        elif mode == "real": self.arucos = arucos

        self.flag_msg = Bool()
        self.array_msg = Float32MultiArray()
        
        self.fiducial_transform = FiducialTransformArray()
        rate = rospy.Rate(int(1.0 / self.dt))

        while not rospy.is_shutdown():
            flag , ar = self.process_transforms()
            self.pub_msgs(flag , ar)
            rate.sleep()

    def process_transforms(self):
        x = 0
        y = 0
        pub_msg = np.array([
            [1, 0.0, 0, 5],
            [0, 1, 0.0, 4],
            [0.0, 0, 1, 2],
            [0.0, 0.0, 0.0, 1.0]])
        if self.fiducial_transform.transforms:
            flag_msg = True
            for arucos in self.fiducial_transform.transforms:
                for posiciones in self.arucos:
                    if arucos.fiducial_id == posiciones.ID:

                        x = arucos.transform.translation.x
                        y = arucos.transform.translation.y
                        z = arucos.transform.translation.z
                        
                        origin2robot = posiciones.transform_robot2aruco(x , y , z)
                        distance , angle_r = self.get_values(origin2robot)

                        comp , _ = self.get_values(pub_msg)
                       
                        if comp > distance:
                            pub_msg = origin2robot
                            d = distance
                            angle = angle_r
                            x = posiciones.x
                            y = posiciones.y
        else:
            flag_msg = False
            angle_r = 0.0
            d = 0
            angle = 0
            
        array = [x , y , d , angle]
        return flag_msg , array
        
    def pub_msgs(self , flag , t):
        self.array_msg.data = t
        self.flag_msg.data = flag

        self.array_pub.publish(self.array_msg)
        self.flag_pub.publish(self.flag_msg)

    def get_values(self , m):
        d = m[:3 , 3]
        distance = np.linalg.norm(d)
        angle_rad = np.arctan2(d[1], d[0])
        return distance , angle_rad
    
    def ft_cb(self, msg):
        self.fiducial_transform = msg

    def cleanup(self):
        print("Apagando Localización")

if __name__ == "__main__":
    ArucoFinder("sim")
