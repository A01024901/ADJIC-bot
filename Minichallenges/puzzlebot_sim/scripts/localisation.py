#!/usr/bin/env python3 

import rospy  

from nav_msgs.msg import Odometry

from std_msgs.msg import Float32 

from tf.transformations import quaternion_from_euler #transformaciones de euler a cuaterniones

from geometry_msgs.msg import PoseStamped

from geometry_msgs.msg import TransformStamped

import tf2_ros

import numpy as np 

 

#This class will do the following: 

#   Subscribe to the /wl and /wr topics

#   Publish localization data to /odom topic

class OdomClass():  

    def __init__(self):  

        # first thing, init a node! 

        rospy.init_node('Localisation') 

        ###******* INIT PUBLISHERS *******###  

        # Create the subscriber to cmd_vel topic 

        self.wr_sub = rospy.Subscriber('wr', Float32, self.wr_cb) # Publisher to wr topic 

        self.wl_sub = rospy.Subscriber('wl', Float32, self.wl_cb) # Publisher to wl topic 

        # Create ROS publishers 

        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=1) #Publisher to pose_sim topic

        ############ ROBOT CONSTANTS ################  

        self.r = 0.05 #puzzlebot wheel radius [m] 

        self.L = 0.19 #puzzlebot wheel separation [m] 

        self.dt = 0.02 # Desired time to update the robot's pose [s] 

        ############ Variables ############### 

        self.w = 0.0 # robot's angular speed [rad/s] 

        self.v = 0.0 #robot's linear speed [m/s] 

        self.x = 0.0 # Initial position of the robot in x [m]

        self.y = 0.0  # Initial position of the robot in y [m]

        self.theta = 0.0  # Initial orientation of the robot [rad]

        self.wr = 0.0 # Right wheel speed

        self.wl = 0.0 # Left wheel speed

 
        self.odom = Odometry() # Create an Odometry message object

        self.tf2_ros = tf2_ros.TransformBroadcaster() # Create a TransformBroadcaster object

        self.t = TransformStamped() # Create a TransformStamped object


        rate = rospy.Rate(int(1.0/self.dt)) # The rate of the while loop will be the inverse of the desired delta_t

        while not rospy.is_shutdown():

            ####### Add / modify  your code here #############


            [self.v, self.w] = self.get_robot_vel(self.wr, self.wl) 

            self.update_robot_pose(self.v, self.w)

            odom = self.get_odom(self.x, self.y, self.theta)

            self.get_transform(self.x, self.y, self.theta)


 
            ######## Publish the data ################# 

            self.odom_pub.publish(odom) 

            rate.sleep() 

     

    def wl_cb(self, msg): 

        self.wl = msg.data

         
    def wr_cb(self, msg): 

        self.wr = msg.data


    def get_robot_vel(self, wr, wl): 

        v = (self.r/2.0)*(wr + wl) # linear speed of the robot

        w = (self.r/self.L)*(wr - wl) # angular speed of the robot

        return [v, w] 

    

    def get_odom(self, x, y, yaw): 

        # x, y and yaw are the robot's position (x,y) and orientation (yaw) 

        # Write the data as a ROS PoseStamped message 

        odom = Odometry()

        odom.header.frame_id = "odom"    #This can be changed in this case I'm using a frame called odom. This is the reference frame of the pose

        self.odom.child_frame_id = "Origin"

        odom.header.stamp = rospy.Time.now() 

        # Position 

        odom.pose.pose.position.x = x # x position of the robot

        odom.pose.pose.position.y = y # y position of the robot

        odom.pose.pose.position.z = 0 # z position of the robot

        # Rotation of the mobile base frame w.r.t. "odom" frame as a quaternion 

        quat = quaternion_from_euler(0,0,yaw) # This function converts the euler angles to quaternions

        odom.pose.pose.orientation.x = quat[0] # x component of the quaternion

        odom.pose.pose.orientation.y = quat[1] # y component of the quaternion

        odom.pose.pose.orientation.z = quat[2] # z component of the quaternion

        odom.pose.pose.orientation.w = quat[3] # w component of the quaternion
        
        return odom
    
    def get_transform(self, x, y, yaw):

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

         

    def update_robot_pose(self, v, w): 

        #This functions receives the robot speed v [m/s] and w [rad/s] 

        # and gets the robot's pose (x, y, theta) where (x,y) are in [m] and (theta) in [rad] 

        # is the orientation,     

        ############ MODIFY THIS CODE   ################

        self.x = self.x + self.dt*v*np.cos(self.theta) # x position of the robot

        self.y = self.y + self.dt*v*np.sin(self.theta)  # y position of the robot

        self.theta = self.theta + self.dt*w # orientation of the robot



############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":
    try:
        OdomClass()
    except rospy.ROSInterruptException:
        pass