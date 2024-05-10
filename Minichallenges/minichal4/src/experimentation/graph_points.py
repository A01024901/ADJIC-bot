#!/usr/bin/env python3 

import rospy 
import os
import numpy as np
import pandas as pd
from geometry_msgs.msg import Pose , PoseArray
from tf.transformations import quaternion_from_euler

class test_mode:
    def __init__(self):
        ###--- Inicio del Nodo ---###
        rospy.init_node('experimental_pub')
        rospy.on_shutdown(self.cleanup)

        ###--- Subscriptores ---###
        ruta = os.path.dirname(os.path.abspath(__file__))

        ###--- Publishers ---###
        self.linear_pub = rospy.Publisher("exp_linear" , PoseArray , queue_size=1)
        self.angular_pub = rospy.Publisher("exp_angular" , PoseArray , queue_size=1)

        ###--- Objects ---###
        self.msg_linear = PoseArray()
        self.msg_angular = PoseArray()
        linear = pd.read_csv(ruta + '/exp_lineares.csv')
        x_linear = linear["Xmedida"] #Variables lineales
        y_linear = linear["Ymedida"]
        angulares = pd.read_csv(ruta + '/exp_angulares.csv')
        ## Agraga el variables angulares
        thetaexp = angulares["Angulos"]

         ###--- Robot Constants ---###
        self.dt = 0.02

        rate = rospy.Rate(int(1.0/self.dt))

        while rospy.get_time() == 0: print ("Simulacion no iniciada") #Descomentar en simulacion 

        print("Simulacion operando")

        for i in range (50):

            ## LINEAR
            linear_pose = Pose()
            linear_pose.position.x = x_linear[i]
            linear_pose.position.y = y_linear[i]
            linear_pose.position.z = 0
            if x_linear[i] == 0:
                theta = 0
            else:
                theta = np.arctan((y_linear[i])/x_linear[i])
            quat = quaternion_from_euler(0.0 , 0.0 ,theta)
            linear_pose.orientation.x = quat[0]
            linear_pose.orientation.y = quat[1]
            linear_pose.orientation.z = quat[2]
            linear_pose.orientation.w = quat[3]
            linear_pose.orientation.w = 1.0

            self.msg_linear.poses.append(linear_pose)

            ## ANGULAR
            
            angular_pose = Pose()
            angular_pose.position.x = 0
            angular_pose.position.y = 0
            angular_pose.position.z = 0
            theta = thetaexp[i]
            quat = quaternion_from_euler(0.0 , 0.0 ,theta)
            angular_pose.orientation.x = quat[0]
            angular_pose.orientation.y = quat[1]
            angular_pose.orientation.z = quat[2]
            angular_pose.orientation.w = quat[3]

            self.msg_angular.poses.append(angular_pose)

        self.msg_angular.header.frame_id = 'odom'
        self.msg_angular.header.stamp = rospy.Time.now()

        self.msg_linear.header.frame_id = 'odom'
        self.msg_linear.header.stamp = rospy.Time.now()    
       

        while not rospy.is_shutdown():
            self.linear_pub.publish(self.msg_linear)
            self.angular_pub.publish(self.msg_angular)
            rate.sleep()


    def cleanup (self):
        print ("Apagando Simulacion")
        

if __name__ == "__main__": 
    test_mode()