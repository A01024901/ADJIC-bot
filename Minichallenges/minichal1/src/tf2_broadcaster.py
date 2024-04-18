#!/usr/bin/env python3 

import rospy 
import tf2_ros
import numpy as np
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler

class tf_broadcaster:
    def __init__(self):
        ###--- Inicio del Nodo ---###
        rospy.init_node('tf2_broadcaster')
        rospy.on_shutdown(self.cleanup)

        ###--- Subscribers ---###
        rospy.Subscriber("wr" , Float32 , self.wr_cb)
        rospy.Subscriber("wl" , Float32 , self.wl_cb)
        rospy.Subscriber("odom" , Odometry , self.odom_cb)

        ###--- Broadcaster ---###
        self.tf_broadcast = tf2_ros.TransformBroadcaster()

        ###--- Transformations ---###
        self.right_wheel = TransformStamped()
        self.left_wheel = TransformStamped()
        self.puzzlebot = TransformStamped()

        ###--- Variables Puzzlebot ---###
        self.x = 0
        self.y = 0
        self.orientation_x = 0
        self.orientation_y = 0
        self.orientation_z = 0
        self.orientation_w = 0

        ###--- Variables Ruedas ---###
        self.theta_r = 0
        self.theta_l = 0
        self.wr = 0
        self.wl = 0

        ###--- Robot Pose ---###
        rate = rospy.Rate(30)

        print("Broadcast operando")
        print("Use rviz to see rotation")

        while not rospy.is_shutdown():

            self.move_puzzlebot('Origin' , 'Bot')
            self.rotate_wheel_r('Bot' , 'right_w')
            self.rotate_wheel_l('Bot' , 'left_w')

            rate.sleep()

    def rotate_wheel_r(self , parent_frame , wheel_frame):
        self.right_wheel.header.stamp = rospy.Time.now() 
        self.right_wheel.header.frame_id = parent_frame
        self.right_wheel.child_frame_id = wheel_frame

        self.right_wheel.transform.translation.x = -1
        self.right_wheel.transform.translation.y = 1

        self.theta_r += self.wr #theta will be increasing 

        #Clip the value of theta to the interval [-pi to pi] 
        self.theta_r = np.arctan2(np.sin(self.theta_r), np.cos(self.theta_r)) 

        #The transformation requires the orientation as a quaternion 
        q = quaternion_from_euler(0, 0, self.theta_r) 
        self.right_wheel.transform.rotation.x = q[0]
        self.right_wheel.transform.rotation.y = q[1]
        self.right_wheel.transform.rotation.z = q[2]
        self.right_wheel.transform.rotation.w = q[3]

        self.tf_broadcast.sendTransform(self.right_wheel) #broadcast the transformation 

    def rotate_wheel_l(self , parent_frame , wheel_frame):
        self.left_wheel.header.stamp = rospy.Time.now() 
        self.left_wheel.header.frame_id = parent_frame
        self.left_wheel.child_frame_id = wheel_frame

        self.left_wheel.transform.translation.x = 1
        self.left_wheel.transform.translation.y = 1

        self.theta_l += self.wl #theta will be increasing 

        #Clip the value of theta to the interval [-pi to pi] 
        self.theta_l = np.arctan2(np.sin(self.theta_l), np.cos(self.theta_l)) 

        #The transformation requires the orientation as a quaternion 
        q = quaternion_from_euler(0, 0, self.theta_l) 
        self.left_wheel.transform.rotation.x = q[0]
        self.left_wheel.transform.rotation.y = q[1]
        self.left_wheel.transform.rotation.z = q[2]
        self.left_wheel.transform.rotation.w = q[3]

        self.tf_broadcast.sendTransform(self.left_wheel) #broadcast the transformation 


    def move_puzzlebot(self , parent_frame , bot_frame):
        self.puzzlebot.header.stamp = rospy.Time.now() 
        self.puzzlebot.header.frame_id = parent_frame
        self.puzzlebot.child_frame_id = bot_frame
        #The transformation requires the orientation as a quaternion 
        self.puzzlebot.transform.rotation.x = self.orientation_x
        self.puzzlebot.transform.rotation.y = self.orientation_y
        self.puzzlebot.transform.rotation.z = self.orientation_z
        self.puzzlebot.transform.rotation.w = self.orientation_w

        self.tf_broadcast.sendTransform(self.puzzlebot) #broadcast the transformation 

    def wr_cb(self , msg):
        self.wr = msg.data

    def wl_cb(self , msg):
        self.wl = msg.data

    def odom_cb (self , msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.orientation_x = msg.pose.pose.orientation.x
        self.orientation_y = msg.pose.pose.orientation.y
        self.orientation_z = msg.pose.pose.orientation.z
        self.orientation_w = msg.pose.pose.orientation.w

    def cleanup (self):
        print ("Esta transmision se ha terminado")    

if __name__ == "__main__": 
    tf_broadcaster()