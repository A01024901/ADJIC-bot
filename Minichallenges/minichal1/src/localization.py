#!/usr/bin/env python3 

import rospy 
import numpy as np
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler
import tf2_ros #ROS package to work with transformations 

class localisation:
    def __init__(self):
        ###--- Inicio del Nodo ---###
        rospy.init_node('localisation')
        rospy.on_shutdown(self.cleanup)

        ###--- Subscriptores ---###
        rospy.Subscriber("/wr" , Float32 , self.wr_cb)
        rospy.Subscriber("/wl" , Float32 , self.wl_cb)

        ###--- Publishers ---###
        self.odom_pub = rospy.Publisher("odom" , Odometry , queue_size=1)
        self.pos_pub = rospy.Publisher("pose_odom" , PoseStamped , queue_size=1)

        ###--- Robot Constants ---###
        self.r = 0.05
        self.l = 0.19
        self.dt = 0.02

        ###--- Variables ---####
        self.w = 0.0
        self.wr = 0.0 
        self.wl = 0.0
        self.v = 0.0
        self.x = 0.0 
        self.y = 0.0
        self.theta = 0.0
        
        self.odom = Odometry()
        rate = rospy.Rate(int(1.0/self.dt))
        self.tf2_ros = tf2_ros.TransformBroadcaster() # Create a TransformBroadcaster object
        self.t = TransformStamped() # Create a TransformStamped object

        #while rospy.get_time() == 0: print ("Simulacion no iniciada")#Descomentar en simulacion 

        print("Nodo operando")

        while not rospy.is_shutdown():
            self.get_robot_velocities()
            self.update_robot_pose()
            self.get_odom()
            self.get_transform(self.x, self.y, self.theta)

            ###--- Publish ---###
            self.odom_pub.publish(self.odom)
            rate.sleep()

    def update_robot_pose(self):
        self.x = self.x + self.v * np.cos(self.theta) * self.dt   
        self.y = self.y + self.v * np.sin(self.theta) * self.dt  
        self.theta = self.theta + self.w * self.dt  
        #print(self.x , self.y , self.theta)           

    def get_robot_velocities (self):
        self.v = (self.r * (self.wr + self.wl))/2
        self.w = self.r * ((((2*self.v/self.r) - self.wl)-self.wl)/self.l)
        #print (self.v , self.w , self.theta)

    def get_odom (self): 
        self.odom.header.frame_id = "Origin"
        self.odom.child_frame_id = "Origin2"
        self.odom.pose.pose.position.x = self.x
        self.odom.pose.pose.position.y = self.y

        quat = quaternion_from_euler(0 , 0 , self.theta)
        self.odom.pose.pose.orientation.x = quat[0]
        self.odom.pose.pose.orientation.y = quat[1]
        self.odom.pose.pose.orientation.z = quat[2]
        self.odom.pose.pose.orientation.w = quat[3]

    def get_transform(self, x, y, yaw):
            # Fill the transformation information 
            self.t.header.stamp = rospy.Time.now() 
            self.t.header.frame_id = "base_link" 
            self.t.child_frame_id = "chassis" 
            self.t.transform.translation.x = x 
            self.t.transform.translation.y = y 
            self.t.transform.translation.z = 0.0 
            # self.theta += 0.01 #theta will be increasing 
            #Clip the value of theta to the interval [-pi to pi] 
            theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))
            #The transformation requires the orientation as a quaternion 
            q = quaternion_from_euler(0, 0, theta) 
            self.t.transform.rotation.x = q[0] 
            self.t.transform.rotation.y = q[1] 
            self.t.transform.rotation.z = q[2] 
            self.t.transform.rotation.w = q[3] 
            # A transformation is broadcasted instead of published 
            self.tf2_ros.sendTransform(self.t) #broadcast the transformation 
        
    def wr_cb (self , msg):
        self.wr = msg.data

    def wl_cb (self , msg):
        self.wl = msg.data

    def cleanup (self):
        print ("Apagando Localsation")
        self.odom_pub.publish(Odometry())
        
        

if __name__ == "__main__": 
    localisation()