#!/usr/bin/env python3
import math
import rospy
import numpy as np
from time import sleep
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, PoseStamped

class Challenge():
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        ### *** CONSTANTS  *** ###
        r = 0.05     # wheel radius (m)
        L = 0.19     # wheel separation (m)
        theta = 0    # angle
        x, y = 0, 0  # position in x and y
        kv, kw = 0.2, 0.6

        self.vel = Twist()
        self.pose = PoseStamped()
        self.X, self.Y = 0.0, 0.0
        self.wr, self.wl = 0.0, 0.0

        ### *** PUBLISHERS  *** ###
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pub_sim = rospy.Publisher('/pose_target', PoseStamped, queue_size=10)

        ### *** SUBSCRIBERS *** ###
        rospy.Subscriber("/pose_sim", PoseStamped, self.get_pose)
        rospy.Subscriber("/wr", Float32, self.get_wr)
        rospy.Subscriber("/wl", Float32, self.get_wl)

        ### **** INIT NODE **** ###
        freq = 20.0
        rate = rospy.Rate(freq)
        dt = 1 / freq  # The time between one calculation and the next one

        while not rospy.is_shutdown():
            v = round(r * (self.wr + self.wl) / 2, 3)
            theta = round(r * (self.wr - self.wl) / L * dt + theta, 3)
            x = x + v * math.cos(theta) * dt
            y = y + v * math.sin(theta) * dt

            if theta > math.pi:
                theta = theta - 2 * math.pi
            elif theta < -math.pi:
                theta = theta + 2 * math.pi

            self.pose.pose.position.x = self.X
            self.pose.pose.position.y = self.Y
            self.pub_sim.publish(self.pose)
            
            # Move robot
            et = round(np.arctan2(self.Y - y, self.X - x) - theta, 3)
            ed = round(math.sqrt((self.X - x)**2 + (self.Y - y)**2), 3)

            self.vel = Twist()
                 
            if et >= 0.25 or -0.25 >= et:
                self.vel.angular.z = kw * et  # w = kw * et
                
            elif ed >= 0.25:
                self.vel.linear.x = kv * ed   # v = kv * ed
                self.vel.angular.z = 0

            self.pub.publish(self.vel)
            self.vel = Twist()
            rate.sleep()

    def get_pose(self, pose):
        """ This function receives a the goal from rviz. """
        self.X = pose.pose.position.x
        self.Y = pose.pose.position.y

    def get_wr(self, wr):
        """ This function receives a the right wheel speed [rad/s] """
        self.wr = wr.data

    def get_wl(self, wl):
        """ This function receives a the left wheel speed [rad/s] """
        self.wl = wl.data

    def cleanup(self):
        """ This function reset data to end the node """
        self.wr = 0.0
        self.wl = 0.0
        self.vel = Twist()
        self.pub.publish(self.vel)
        print("That's all, bye 🦀")

if __name__ == "__main__":
    rospy.init_node("puzzlebot_kinematic_model", anonymous=True)
    Challenge()
