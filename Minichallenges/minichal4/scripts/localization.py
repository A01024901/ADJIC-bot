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
        #self.pos_pub = rospy.Publisher("pose_odom" , PoseStamped , queue_size=1)

        ###--- Robot Constants ---###
        self.r = 0.05
        self.l = 0.19
        self.dt = 0.02

        self.mu = np.array([[0.0 , 0.0 , 0.0]])
        self.Q = np.array([[0.001 , 0.001 , 0.002] , [0.0001 , 0.002 , 0.0001] , [0.0002 , 0.0001 , 0.001]])
        self.sigma = np.eye(3)

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


            self.H = np.array([[1, 0, -self.dt * self.v * np.sin(self.theta[2,0])], [0, 1, self.dt * self.v * np.cos(self.theta[2,0])], [0, 0, 1]]) # Jacobian of the measurement model
            self.mu = np.array([[self.mu[0] + self.dt * self.v * np.cos(self.mu[2,0])], [self.mu[1] + self.dt * self.v * np.sin(self.mu[2])], [self.mu[2] + self.v * self.w]]) # Prediction step
            self.sigma = self.H.dot(self.sigma).dot(self.H.T) + self.Q # Covariance matrix update

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
        self.odom.header.frame_id = "odom"
        #self.odom.child_frame_id = "Origin2"
        self.odom.pose.pose.position.x = self.x
        self.odom.pose.pose.position.y = self.y

        quat = quaternion_from_euler(0 , 0 , self.theta)
        self.odom.pose.pose.orientation.x = quat[0]
        self.odom.pose.pose.orientation.y = quat[1]
        self.odom.pose.pose.orientation.z = quat[2]
        self.odom.pose.pose.orientation.w = quat[3]

        self.odom.twist.twist.linear.x = self.v
        self.odom.twist.twist.angular.z = self.w

        self.odom.pose.covariance = [0,0] * 36
        self.odom.pose.covariance[0] = self.sigma[0],[0] * self.sigma[0][0]
        self.odom.pose.covariance[1] = self.sigma[0][1]
        self.odom.pose.covariance[5] = self.sigma[0][2]
        self.odom.pose.covariance[6] = self.sigma[1][0]
        self.odom.pose.covariance[7] = self.sigma[1][2]
        self.odom.pose.covariance[11] = self.sigma[1][2]
        self.odom.pose.covariance[30] = self.sigma[2][0]
        self.odom.pose.covariance[31] = self.sigma[2][1]
        self.odom.pose.covariance[35] = self.sigma[2][2] * self.sigma[2][2]
        

        self.odom_pub.publish(self.odom)

    def get_transform(self, x, y, yaw, Sigma):
            # Fill the transformation information 
            self.t.header.stamp = rospy.Time.now() 
            self.t.header.frame_id = "odom" 
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