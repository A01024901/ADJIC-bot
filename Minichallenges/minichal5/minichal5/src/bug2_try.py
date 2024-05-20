#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class BUG_2():  
    def __init__(self) :
        # INITIATING THE NODE
        rospy.init_node("bug_2")
        print("BUG 2 Node Initialized")

        rospy.on_shutdown(self.shutdown) 

        # PUBLISHERS
        self.pub_cmd_vel = rospy.Publisher("/puzzlebot_1/base_controller/cmd_vel", Twist, queue_size = 1)

        # SUBSCRIBERS
        rospy.Subscriber("/puzzlebot_1/wl", Float32, self.wl_cb)  
        rospy.Subscriber("/puzzlebot_1/wr", Float32, self.wr_cb)
        rospy.Subscriber("/puzzlebot_1/scan", LaserScan, self.lidar_cb)

        # ROBOT CONSTANTS
        self.r = 0.05               # Puzzlebot wheel radius [m] 
        self.L = 0.19               # Puzzlebot wheel separation [m] 
        self.dt = 0.02              # Desired time to update the robot's pose [s] 

        # VARIABLES
        w = 0.0                     # Robot's angular speed [rad / s]
        v = 0.0                     # Robot's linear speed [rad / s]

        goal_X = 2.70               # Goal position in X
        goal_Y = -1.0                # Goal position in Y
        goal_THETA = 0.0 
        error_THETA = 0.0

        vel = Twist()
        # self.lidar = LaserScan()

        self.wl = 0.0               # Left wheel speed [rad / s]
        self.wr = 0.0               # Right wheel speed [rad / s]
        
        self.robot_X = 0.0          # Robot x position [m]
        self.robot_Y = 0.0          # Robot y position [m]
        self.robot_THETA = 0.0      # Robot angle position [rad]

        k_v = 0.25
        k_w = 0.7
        k_w_AO = 0.5
        k_w_C_AO = 3.0
        k_c = 0.1

        v_C_AO = 0.15
        w_AO = 0.0
        v_GTG = 0.0
        w_GTG = 0.0

        d_min = 0.07                # Tolerated Distance to the Goal [m]
        epsilon = 0.1
        fw_THETA = 0.0
        stopping_d = 0.17
        avoiding_d = 0.23
        self.closest_d = 0.0
        self.hit_point_d = 0.0
        hit_point_reached = False
        self.avoiding_THETA = 0.0

        self.state = "STOP"
        self.lidar_received = 0

        rate = rospy.Rate(int(1.0 / self.dt))

        while rospy.get_time() == 0 :
            print("NO simulated time has been received")
        print("TIME RECEIVED")

        prev_time = rospy.get_time()

        while not rospy.is_shutdown() :
            self.dt = rospy.get_time() - prev_time
            prev_time = rospy.get_time()

            v = self.r * ((self.wl + self.wr) / 2.0)
            w = self.r * ((self.wr - self.wl) / self.L)

            self.robot_THETA = self.robot_THETA + (w * self.dt)
            self.robot_THETA = np.arctan2(np.sin(self.robot_THETA), np.cos(self.robot_THETA))
            self.robot_X = self.robot_X + (v * np.cos(self.robot_THETA) * self.dt)
            self.robot_Y = self.robot_Y + (v * np.sin(self.robot_THETA) * self.dt)

            d = np.sqrt(((goal_X - self.robot_X) ** 2) + ((goal_Y - self.robot_Y) ** 2))
            goal_THETA = np.arctan2((goal_Y - self.robot_Y), (goal_X - self.robot_X))
            error_THETA = goal_THETA - self.robot_THETA
            error_THETA = np.arctan2(np.sin(error_THETA), np.cos(error_THETA))  

            if self.lidar_received :
                self.lidar_received = 0
                if self.state == "STOP" :
                    if d > d_min :
                        self.state = "GO TO GOAL"
                    else :
                        print("STOP STATE\n")
                        vel.linear.x = 0.0
                        vel.angular.z = 0.0
                elif self.state == "GO TO GOAL" :  
                    fw_THETA = (np.pi / 2) + self.avoiding_THETA 
                    fw_THETA = np.arctan2(np.sin(fw_THETA), np.cos(fw_THETA))  
                    if (self.closest_d < avoiding_d) and (abs(fw_THETA - goal_THETA) <= (np.pi / 2.0) - 0.2) :
                        self.state = "AVOIDING OBSTACLE CCW"
                        self.hit_point_d = d
                        hit_point_reached = True
                    elif (self.closest_d < avoiding_d) and (abs(fw_THETA - goal_THETA) > (np.pi / 2.0) - 0.2) :
                        self.state = "AVOIDING OBSTACLE CW"
                        self.hit_point_d = d
                        hit_point_reached = True
                    elif d < d_min :
                        self.state = "STOP"
                    else :
                        v_GTG = k_v * d

                        if self.closest_d <= 0.4 :
                            v_GTG = self.closest_d * k_c
                        
                        w_GTG = k_w * error_THETA

                        vel.linear.x = v_GTG
                        vel.angular.z = w_GTG

                        print("MOVING TO THE GOAL")
                        print("Robot X position: " + str(self.robot_X))
                        print("Robot Y position: " + str(self.robot_Y))
                        print("Robot angle: " + str(self.robot_THETA) + '\n')
                        print("Distance: " + str(d))
                        print("ERROR-Theta: " + str(error_THETA) + '\n')
                elif self.state == "AVOIDING OBSTACLE CCW" :
                    if ((d < (self.hit_point_d - epsilon)) and 
                            ((abs(self.avoiding_THETA - goal_THETA) < (np.pi / 2.0)) or 
                                self.on_mline(goal_X, goal_Y))) :
                        self.state = "GO TO GOAL"
                        hit_point_reached = False
                    elif (d < d_min) or (self.closest_d < stopping_d) :
                        print("COMPROMISED SAFE ZONE")
                        self.state = "STOP"
                    else :  
                        fw_THETA = (np.pi / 2) + self.avoiding_THETA

                        if (-0.12 > fw_THETA > 0.12) and hit_point_reached :
                            v_AO = 0.0
                            w_AO = k_w_AO * fw_THETA
                        else :
                            v_AO = v_C_AO
                            w_AO = k_w_C_AO * fw_THETA

                        vel.linear.x = v_AO
                        vel.angular.z = w_AO

                        print("AVOIDING THE OBSTACLE CCW")
                        print("Robot X position: " + str(self.robot_X))
                        print("Robot Y position: " + str(self.robot_Y))
                        print("Robot angle: " + str(self.robot_THETA) + '\n')
                        print("Distance: " + str(d))
                        print("ERROR-Theta: " + str(error_THETA) + '\n')
                elif self.state == "AVOIDING OBSTACLE CW" :
                    if ((d < (self.hit_point_d - epsilon)) and 
                            ((abs(self.avoiding_THETA - goal_THETA) < (np.pi / 2.0)) or 
                                self.on_mline(goal_X, goal_Y))) :
                        self.state = "GO TO GOAL"
                        hit_point_reached = False
                    elif (d < d_min) or (self.closest_d < stopping_d) :
                        print("COMPROMISED SAFE ZONE")
                        self.state = "STOP"
                    else :    
                        fw_THETA = -(np.pi / 2) + self.avoiding_THETA

                        print('\nfw_t: ', fw_THETA)

                        if (-0.12 > fw_THETA > 0.12) and hit_point_reached :
                            v_AO = 0.0
                            w_AO = k_w_AO * fw_THETA
                        else :
                            v_AO = v_C_AO
                            w_AO = k_w_C_AO * fw_THETA

                        vel.linear.x = v_AO
                        vel.angular.z = w_AO

                        print("AVOIDING THE OBSTACLE CW")
                        print("Robot X position: " + str(self.robot_X))
                        print("Robot Y position: " + str(self.robot_Y))
                        print("Robot angle: " + str(self.robot_THETA) + '\n')
                        print("Distance: " + str(d))
                        print("ERROR-Theta: " + str(error_THETA) + '\n')
                else :
                    print("OUT OF STATE MACHINE\n")
                    vel.linear.x = 0.0
                    vel.angular.z = 0.0

            self.pub_cmd_vel.publish(vel)  
            rate.sleep()

    def wl_cb(self, wl) :  
        self.wl = wl.data 
         
    def wr_cb(self, wr) :  
        self.wr = wr.data

    def on_mline(self, goal_X, goal_Y) :
        x0, y0 = (0, 0)         # <--- START POSITION
        
        A = goal_Y - y0
        B = x0 - goal_X
        C = (goal_X * x0) - (y0 * goal_Y)

        d = abs((A * self.robot_X) + (B * self.robot_Y) + C) / np.sqrt((A ** 2) + (B ** 2))

        return d < 0.1

    def lidar_cb(self, lidar) :
        self.lidar_received = 1
        ranges = np.array(lidar.ranges)
        valid_ranges = ranges[np.isfinite(ranges)]
        self.closest_d = valid_ranges.min() if valid_ranges.size > 0 else float('inf')
        idx = ranges.tolist().index(self.closest_d) if valid_ranges.size > 0 else 0
        self.avoiding_THETA = lidar.angle_min + idx * lidar.angle_increment
        self.avoiding_THETA = np.arctan2(np.sin(self.avoiding_THETA), np.cos(self.avoiding_THETA))

    def shutdown(self) :    
        stop_msg = Twist() 
        self.pub_cmd_vel.publish(stop_msg)
        print("BUG 2 Node Killed")
 
if __name__ == "__main__":  
    BUG_2()
