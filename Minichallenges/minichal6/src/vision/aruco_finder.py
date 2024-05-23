#!/usr/bin/env python3 

import rospy 
import numpy as np
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler
import tf2_ros #ROS package to work with transformations 
from fiducial_msgs.msg import F

class localisation:
    def __init__(self):
        ###--- Inicio del Nodo ---###
        rospy.init_node('aruco_finder')
        rospy.on_shutdown(self.cleanup)

        ###--- Subscriptores ---###
        rospy.Subscriber("/wr" , Float32 , self.wr_cb)

        ###--- Publishers ---###
        self.odom_pub = rospy.Publisher("odom" , Odometry , queue_size=1)

        ###--- Robot Constants ---###
        self.dt = 0.02

        ###--- Variables ---####
        
        ###--- Objects ---###
        rate = rospy.Rate(int(1.0/self.dt))

        while rospy.get_time() == 0: pass #Descomentar en simulacion 

        print("Nodo operando")

        while not rospy.is_shutdown():
            rate.sleep()


    def wl_cb (self , msg):
        self.wl = msg.data

    def cleanup (self):
        print ("Apagando Localsation")
        self.odom_pub.publish(Odometry())
        
        

if __name__ == "__main__": 
    localisation()