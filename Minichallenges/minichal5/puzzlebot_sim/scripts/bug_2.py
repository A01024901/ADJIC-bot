#!/usr/bin/env python3 

import rospy  
import numpy as np 
from sensor_msgs.msg import LaserScan   
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist 

# This class implements a simple obstacle avoidance algorithm 
class WallFollowingControlClass():  
    def __init__(self):  
        rospy.on_shutdown(self.cleanup)  
        ####################### PUBLISEHRS AND SUBSCRIBERS ############################  
        rospy.Subscriber("scan", LaserScan, self.laser_cb) #base_scan
        rospy.Subscriber("gtg_vel", Twist, self.gtg_cb)
        rospy.Subscriber("ed", Float32, self.ed_cb)
        rospy.Subscriber("ed_theta", Float32, self.ed_theta_cb) 
        rospy.Subscriber("goal_x", Float32, self.gx_cb)
        rospy.Subscriber("goal_y", Float32, self.gy_cb)
        rospy.Subscriber("xp", Float32, self.xp_cb)
        rospy.Subscriber("yp", Float32, self.yp_cb)
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1) 

        ######################## CONSTANTS AND VARIABLES ############################## 
        self.laser_received             = False 
        self.control_received           = False
        self.avoid_obstacle_received    = False
        self.ed_received                = False
        self.ed_theta_received          = False
        self.flag_wf                    = True
        self.flag                       = "go to goal"
        self.gx_received = False
        self.gy_received = False
        self.xp_received = False
        self.yp_received = False
        self.flag_current_state         = "gtg"
        # v_desired  = 0.4 #[m/s] desired speed when there are no obstacles 
        self.closest_angle = 0.0     # Angle to the closest object 
        self.closest_range = np.inf  # Distance to the closest object 
        vel_msg = Twist() 
        # Speeds from topics
        self.ao_msg     = Twist()
        self.gtg_msg    = Twist()
        self.ed_msg     = Float32()
        self.ed_theta_msg     = Float32()
        # Desired goal and actual pos
        self.goal_x, self.goal_y    = 0.0, 0.0
        self.xp, self.yp            = 0.0, 0.0
        epsilon     = 0.3 
        self.ed_tau = 0.0
        self.distance_H = 0.0   # Distance to HIT POINT
        self.distance_L = 0.0   # Distance to LEAVE POINT
        rate = rospy.Rate(10) #10Hz is the lidar's frequency  
        print("Node initialized 1hz") 

        ############################### MAIN LOOP ##################################### 
        while not rospy.is_shutdown():  
            ############################### YOUR CODE HERE ############################## 
            x0, y0 = 0.0, 0.0                   # Start position
            x1, y1 = self.xp, self.yp           # Actual position
            x2, y2 = self.goal_x, self.goal_y   # End/goal position
            if self.laser_received and self.control_received and self.ed_received and self.ed_theta_received and self.gx_received and self.gy_received and self.xp_received and self.yp_received:
                self.laser_received = False
                # dfw = min(self.lidar_msg.ranges)
                vel_msg = Twist()
                # Closest range and theta_ao
                l_msg = self.lidar_msg
                # Transform laser data because of <laser> PI rotation w.r.t. <base_link>
                new_ranges = self.tf_laser_data(l_msg.ranges)
                closest_range = min(new_ranges)
                idx = l_msg.ranges.index(closest_range) 
                closest_angle = l_msg.angle_min + idx * l_msg.angle_increment   # closest_angle
                theta = closest_angle
                theta       = np.arctan2(np.sin(theta),np.cos(theta))
                theta_ao    = theta - np.pi
                theta_ao    = np.arctan2(np.sin(theta_ao),np.cos(theta_ao))     # theta_ao
                vel_gtg     = self.gtg_msg.linear.x
                w_gtg       = self.gtg_msg.angular.z
                theta_gtg = self.ed_theta_msg.data
            
                x0, y0 = 0.0, 0.0                   # Start position
                x1, y1 = self.xp, self.yp           # Actual position
                x2, y2 = self.goal_x, self.goal_y   # End/goal position
                goal_distance = self.ed_msg.data    # Distance to goal [m]
                fw_distance = 0.4                   # Following wall distance [m] -- # 0.4
                target_position_tolerance = 0.20    # Target position tolerance [m] 
                
                print("================ INIT ================")
                if (goal_distance < target_position_tolerance):      # At goal - Goal reached
                    print("Goal reached STOP")
                    self.flag_current_state = "stop"
                    #print("Stop")
                    vel_msg.linear.x, vel_msg.angular.z = 0.0, 0.0

                elif self.flag_current_state == "gtg":                 # Go to goal
                    if closest_range <= fw_distance: 
                        # Implement the following walls behavior
                        self.distance_H = goal_distance                # Save HIT POINT distance to goal
                        print(" change to following walls") 
                        self.flag_current_state = "Clockwise"                    

                    else:
                        print("Moving to the Goal") 
                        vel_msg.linear.x, vel_msg.angular.z = vel_gtg, w_gtg 

                elif self.flag_current_state == "Clockwise":
                    self.distance_L = goal_distance                    # Save LEAVE POINT distance to goal EVERY TIME we are in WF behaviour
                    # min dist bet point and line
                    min_d = self.min_dist_bet_two_p(x0, y0, x1, y1, x2, y2)
                    thresh_min_d = 0.10

                    if (((min_d >= 0.0) and (min_d <= thresh_min_d)) and (self.distance_L < abs(self.distance_H - epsilon))):
                        # Output conditions of Wall Following behaviour ---> FAT GUARDS and MIN DISTANCE LINE
                        self.flag_current_state = "gtg"
                        print("Change to Go to goal") 

                    else:
                        # Always one way
                        flag_cc_ccc = 1     # 0 --> Clockwise
                                            # 1 --> Counterclockwise
                        if flag_cc_ccc == 0:    theta_fw  = -np.pi/2 + theta_ao  # Clockwise behaviour
                        else:                   theta_fw  = np.pi/2 + theta_ao   # Counterclockwise behaviour
                        theta_fw = np.arctan2(np.sin(theta_fw), np.cos(theta_fw))
                        
                        # Let the algorithm decide
                        Kw = 1.6 
                        vel_msg.linear.x, vel_msg.angular.z = 0.12, Kw * theta_fw

                elif self.flag_current_state == 'stop': 
                    print("STOP")
                    vel_msg.linear.x, vel_msg.angular.z = 0.0, 0.0
                
                print(f"""Actual State:         {self.flag_current_state}
vel_x:                {str(vel_msg.linear.x)}
vel_z:                {str(vel_msg.angular.z)}
closest_range <= 0.4: {str(closest_range)}
goal_distance or LP:  {str(goal_distance)}
GD or LP < THIS (FG): {str(abs(self.distance_H - epsilon))}
1.5708   > THIS (CS): {str(abs(theta_ao - theta_gtg))}
dis_min_line:         {str( self.min_dist_bet_two_p(x0, y0, x1, y1, x2, y2) )}
================ END ================""")

            self.cmd_vel_pub.publish(vel_msg)
            rate.sleep()  

    def gx_cb(self, msg):
        self.goal_x = msg.data
        self.gx_received = True
        
    def gy_cb(self, msg):
        self.goal_y = msg.data
        self.gy_received = True
        
    def xp_cb(self, msg):
        self.xp = msg.data
        self.xp_received = True
    
    def yp_cb(self, msg):
        self.yp = msg.data
        self.yp_received = True
    
    def dist_bet_two_p(self, x2, y2, x1, y1):
        res = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        return res
        
    def min_dist_bet_two_p(self, x0, y0, x1, y1, x2, y2):
        # function
        # ============
        # y = a*x + b
        # y = (y2-y1)/(x2-x1) * x + (x2*y1 - x1*y2)/(x2 - x1)
        # (y2-y1)/(x2-x1) * X - Y + (x2*y1 - x1*y2)/(x2 - x1)
        a = (y2-y0)/(x2-x0)
        b = -1
        c = (x2*y0 - x0*y2)/(x2 - x0)
        d = (abs(a*x1 + b*y1 + c))/(np.sqrt(a**2 + b**2))
        print(d)
        return d
    
    def tf_laser_data(self, lidar_ranges):
        ori = lidar_ranges
        # len(a) = 1147 impar -- mid point (in front) -- index int(1147/2 - 0.5)
        fin = np.roll(ori, int(len(lidar_ranges)/2 + 1))              
        return fin

    def laser_cb(self, msg):  
        # This function receives a message of type LaserScan and computes the closest object direction and range 
        self.lidar_msg = msg 
        self.laser_received = True
        
    def gtg_cb(self, msg):
        self.gtg_msg = msg
        self.control_received = True
        
    def ed_cb(self, msg):
        self.ed_msg = msg
        self.ed_received = True
        
    def ed_theta_cb(self, msg):
        self.ed_theta_msg = msg
        self.ed_theta_received = True

    def cleanup(self):  
        # This function is called just before finishing the node  
        # You can use it to clean things up before leaving  
        # Example: stop the robot before finishing a node.    
        vel_msg = Twist() 
        self.cmd_vel_pub.publish(vel_msg)  

############################### MAIN PROGRAM ####################################  
if __name__ == "__main__":  
    rospy.init_node("wall_following", anonymous=True)  
    WallFollowingControlClass() 

 
