#!/usr/bin/env python3 

import rospy 
import numpy as np
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from tf.transformations import quaternion_from_euler

class puzzlebot_kinemactic_model:
    def __init__(self):
        ###--- Inicio del Nodo ---###
        rospy.init_node('puzzlebot_kinematic_model')
        rospy.on_shutdown(self.cleanup)

        ###--- Subscriptores ---###
        rospy.Subscriber("/cmd_vel" , Twist , self.vel_cb)

        ###--- Publishers ---###
        self.pose_pub = rospy.Publisher("pose_sim" , PoseStamped , queue_size=1)
        self.wr_pub = rospy.Publisher("wr" , Float32 , queue_size=1)
        self.wl_pub = rospy.Publisher("wl" , Float32 , queue_size=1)

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

        self.pose_stamped = PoseStamped()
        rate = rospy.Rate(int(1.0/self.dt))

        #while rospy.get_time() == 0: print ("Simulacion no iniciada")#Descomentar en simulacion 

        print("Simulacion operando")

        while not rospy.is_shutdown():
            
            self.update_robot_pose()
            pose_stamped = self.get_pose_stamped(self.x , self.y , self.theta)
            self.get_wheel_speed()

            ###--- Publish ---###
            self.pose_pub.publish(pose_stamped)
            self.wr_pub.publish(self.wr)
            self.wl_pub.publish(self.wl)
            rate.sleep()

    def update_robot_pose(self ):
        self.x = self.x + self.v * np.cos(self.theta) * self.dt   
        self.y = self.y + self.v * np.sin(self.theta) * self.dt  
        self.theta = self.theta + self.w * self.dt             

    def get_wheel_speed(self):
        self.wl = (2 * self.v - self.w * self.l)/(self.r * 2)
        self.wr = (2 * self.v +  self.w * self.l)/(self.r * 2)
        #print (self.wr , self.wl , self.theta) 
    
    def get_pose_stamped(self , x , y , yaw):
        ###--- Write Pose_stamped message ---###
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "Origin"
        pose_stamped.header.stamp = rospy.Time.now()

        ###--- Position ---###
        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y

        ###--- Rotation ---###
        quat = quaternion_from_euler(0 , 0 , yaw)
        pose_stamped.pose.orientation.x = quat[0]
        pose_stamped.pose.orientation.y = quat[1]
        pose_stamped.pose.orientation.z = quat[2]
        pose_stamped.pose.orientation.w = quat[3]

        return pose_stamped 
        
    def vel_cb (self , msg):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def cleanup (self):
        print ("Apagando Simulacion")
        self.pose_pub.publish(PoseStamped())
        self.wr_pub.publish(0.0)
        self.wl_pub.publish(0.0)
        
        

if __name__ == "__main__": 
    puzzlebot_kinemactic_model()