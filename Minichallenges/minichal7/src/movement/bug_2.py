#!/usr/bin/env python3  

import rospy  
from geometry_msgs.msg import Twist, PoseStamped 
from std_msgs.msg import Float32 
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion 
from sensor_msgs.msg import LaserScan   
import numpy as np 

#This class will make the puzzlebot move to a given goal 
class AutonomousNav():  
    def __init__(self): 
        rospy.init_node('bug_2') 
        rospy.on_shutdown(self.cleanup)

        ###--- Subscriptores ---###
        rospy.Subscriber("puzzlebot_1/wl", Float32, self.wl_cb)  
        rospy.Subscriber("puzzlebot_1/wr", Float32, self.wr_cb)  
        rospy.Subscriber("puzzlebot_1/scan", LaserScan, self.laser_cb) 
        rospy.Subscriber("puzzlebot_1/base_controller/odom", Odometry, self.odom_cb)

        ###--- Publishers ---###
        self.pub_cmd_vel = rospy.Publisher('/puzzlebot_1/base_controller/cmd_vel', Twist, queue_size=1)  

        ###--- Robot Constants ---###
        self.dt = 0.02
        self.r = 0.05 #wheel radius [m] 
        self.L = 0.19 #wheel separation [m] 
        fw_distance = 0.3
        stop_distance = 0.001 # distance from closest obstacle to stop the robot [m] 
        self.target_position_tolerance=0.01 #acceptable distance to the goal to declare the robot has arrived to it [m] 

        ###--- Variables ---####
        targets = [[0 , 0] , [0 , 0] , [0 , 0] , [0 , 0] , [0 , 0]]
        self.x_target = 0.0 #x position of the goal 
        self.y_target = 0.0 #y position of the goal 
        self.lidar_received = 0 #flag to indicate if the laser scan has been received
        self.integral = 0.0
        self.prev_error = 0.0
        
        self.wr=0 #right wheel speed [rad/s] 
        self.wl=0 #left wheel speed [rad/s] 
        self.current_state = 'GoToGoal' #Robot's current state 

        ###---- Robot Pose
        self.x = 0.0 #x position of the robot [m] 
        self.y = 0.0 #y position of the robot [m] 
        self.theta = 0.0 #angle of the robot [rad] 

        ###--- Objetos ---####
        rate = rospy.Rate(int(1.0/self.dt))
        v_msg = Twist() #Robot's desired speed  

        while not rospy.is_shutdown():
            if self.lidar_received:
                closest_range, closest_angle = self.get_closest_object(self.lidar_msg) #get the closest object range and angle 

                if self.current_state == 'Stop': 
                    if self.goal_received: 
                        print("Change to Go to goal from stop") 
                        self.current_state = "GoToGoal" 
                        self.goal_received=0 

                    else: 
                        v_msg.linear.x = 0.0 
                        v_msg.angular.z = 0.0 

                elif self.current_state == 'GoToGoal':  
                    if self.at_goal() or  closest_range <  stop_distance:  
                        print("Change to Stop from Go to goal") 
                        print(self.x_target)
                        print(self.y_target)
                        self.current_state = "Stop"  

                    elif closest_range < fw_distance: 
                        print("Change to wall following from Go to goal") 
                        self.current_state= "WallFollower" 

                    else:        
                        v_gtg, w_gtg = self.compute_gtg_control(self.x_target, self.y_target, self.x, self.y, self.theta)   
                        v_msg.linear.x = v_gtg 
                        v_msg.angular.z = w_gtg      

                elif self.current_state == 'WallFollower': 
                    theta_gtg, theta_ao = self.compute_angles(self.x_target, self.y_target, self.x, self.y, self.theta, closest_angle)
                    d_t = np.sqrt((self.x_target-self.x)*2+(self.y_target-self.y)*2)
                    print("Quit?: ", self.quit_fw_bug_two(theta_gtg, theta_ao, self.x_target, self.y_target, self.x, self.y))

                    if self.at_goal() or closest_range < stop_distance: 
                        print("Change to Stop") 
                        self.current_state = "Stop" 
                        ffw = True

                    elif self.quit_fw_bug_two(theta_gtg, theta_ao, self.x_target, self.y_target, self.x, self.y):
                        print("Change to Go to goal from wall follower") 
                        self.current_state = "GoToGoal" 
                        ffw = True

                    else:
                        d_t1 = np.sqrt((self.x_target-self.x)*2+(self.y_target-self.y)*2)  if ffw else d_t1
                        clk_cnt = self.clockwise_counter(self.x_target, self.y_target, self.x, self.y, self.theta, closest_angle) if ffw else clk_cnt
                        ffw = False
                        v_fw, w_fw = self.following_walls(closest_angle, clk_cnt) 
                        v_msg.linear.x = v_fw
                        v_msg.angular.z = w_fw 

                rate.sleep()
            self.pub_cmd_vel.publish(v_msg)              
              

    def laser_cb(self, msg):   
        ## This function receives a message of type LaserScan   
        self.lidar_msg = msg  
        self.lidar_received = 1  

    def wl_cb(self, wl):  
        ## This function receives a the left wheel speed [rad/s] 
        self.wl = wl.data 

    def wr_cb(self, wr):  
        ## This function receives a the right wheel speed.  
        self.wr = wr.data  

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, self.theta) = euler_from_quaternion(orientation_list)

        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta)) 

    def cleanup(self):  
        vel_msg = Twist() 
        self.pub_cmd_vel.publish(vel_msg) 

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":   
    AutonomousNav()
        