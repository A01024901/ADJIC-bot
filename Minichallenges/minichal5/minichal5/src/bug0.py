#!/usr/bin/env python3 

import rospy 
import numpy as np
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

class follow_walls:
    def __init__(self):
        ###--- Inicio del Nodo ---###
        rospy.init_node('follow_wall')
        rospy.on_shutdown(self.cleanup)

        ###--- Subscriptores ---###
        rospy.Subscriber("/puzzlebot_1/scan", LaserScan, self.laser_cb)
        rospy.Subscriber("pos_gtg" , PoseStamped , self.pos_gtg_cb) 

        ###--- Publishers ---###
        self.pub_cmd_vel = rospy.Publisher('gtg_twist', Twist, queue_size=1) 
        self.pub_flag_front = rospy.Publisher('front_object', Twist, queue_size=1) 

        ###--- Constants ---###
        self.follow_distance = 0.25
        self.dt = 0.02

        ###--- Objetos ---###
        self.cmd_vel = Twist() 
        self.scan = LaserScan()
        self.object_flag = Bool()
        rate = rospy.Rate(int(1.0/self.dt))

        ###--- Variables ---###
        self.scan_flag = False
        self.x_pos = 0
        self.y_pos = 0
        

        while rospy.get_time() == 0: print ("Simulacion no iniciada") 

        while not rospy.is_shutdown():
            if self.scan.ranges:
                self.scan_flag = False
                closest_distance = min(self.scan)
                index = self.scan.ranges.index(closest_distance)
                closest_angle = self.scan.angle_min + index * self.scan.angle_increment

                self.object_flag.data = self.eval_conditions(closest_distance , closest_angle)     

            ###--- Publish ---###
            self.pub_cmd_vel.publish(self.cmd_vel)
            self.pub_flag_front.publish(self.object_flag)
            rate.sleep()

    
    def laser_cb (self , msg):
        self.scan = msg
        data = msg.ranges
        self.scan.ranges = np.roll(data, int(len(data)/2 + 1))   

    def eval_conditions(self , distance , angle):
        distance = distance <= self.follow_distance
        angle = angle > -45.0 and angle < 45.0
        if distance and angle: return True
        else: return False

    def wl_cb (self , msg):
        self.wl = msg.data

    def pos_gtg_cb(self , msg):
        self.x_pos = msg.pose.position.x 
        self.y_pos = msg.pose.position.y

    def cleanup (self):
        print ("Apagando Localsation")
        self.pub_cmd_vel.publish(Twist())
        self.pub_flag_front.publish(Bool())
        
        
if __name__ == "__main__": 
    follow_walls()