#!/usr/bin/env python3 

import rospy 
import numpy as np
from fiducial_msgs.msg import FiducialTransformArray
from std_msgs.msg import Bool
from std_msgs.msg import Float32

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
        
<<<<<<< HEAD
    def transform_origin2robot(self , x , y  , z):
=======
    def get_trasnform(self , x , y , z):
>>>>>>> 8509bb812127266eb42d7e1358d000703f27dcdc
        camera2aruco = np.array([
            [1, 0.0, 0, x],
            [0, 1, 0.0, y],
            [0.0, 0, 1, z],
            [0.0, 0.0, 0.0, 1.0]])
        
        aruco2camera = self.get_inv(camera2aruco)
        camera2robot = self.get_inv(self.robot2camera)
        
        self.origin2robot = self.origin2aruco.dot(aruco2camera).dot(camera2robot)

        return self.origin2robot
    
    def transform_robot2aruco(self):
        pass

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
        self.x_pub = rospy.Publisher("/ar_x" , Float32 , queue_size=1)
        self.y_pub = rospy.Publisher("/ar_y" , Float32 , queue_size=1)
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

        self.x_msg = Float32()
        self.y_msg = Float32()
        self.flag_msg = Bool()
        
        self.fiducial_transform = FiducialTransformArray()
        rate = rospy.Rate(int(1.0 / self.dt))

        while not rospy.is_shutdown():
<<<<<<< HEAD
            flag , t = self.process_transforms()
            self.pub_msgs(flag , t)
            rate.sleep()

    def process_transforms(self):
=======
            self.process_transforms()
            rate.sleep()

    def process_transforms(self):
        d = 0
>>>>>>> 8509bb812127266eb42d7e1358d000703f27dcdc
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
                        print("Fiducial ID: ", posiciones.ID)

<<<<<<< HEAD
                        x = arucos.transform.translation.x
=======
                        x = arucos.transform.translation.x 
>>>>>>> 8509bb812127266eb42d7e1358d000703f27dcdc
                        y = arucos.transform.translation.y
                        z = arucos.transform.translation.z
                       
                        
<<<<<<< HEAD
                        origin2robot = posiciones.transform_origin2robot(x , y , z)
=======
                        origin2robot = posiciones.get_trasnform(x , y , z)
                       
>>>>>>> 8509bb812127266eb42d7e1358d000703f27dcdc
                        distance = self.get_distance(origin2robot)
                        
                       
                        
                        if self.get_distance(pub_msg) > distance:
                            pub_msg = origin2robot
                            d = distance
        else:
            flag_msg = False
            

        return flag_msg , pub_msg 

        
    def pub_msgs(self , flag , t):
        self.x_msg.data = t[0][3]
        self.y_msg.data = t[1][3]
        self.flag_msg.data = flag

        self.x_pub.publish(self.x_msg)
        self.y_pub.publish(self.y_msg)
        self.flag_pub.publish(self.flag_msg)


    def get_distance(self , m):
        d = m[:3 , 3]
        distance = np.linalg.norm(d)
        return distance
    
    def ft_cb(self, msg):
        self.fiducial_transform = msg

    def cleanup(self):
        print("Apagando Localización")

if __name__ == "__main__":
    ArucoFinder("sim")
