#!/usr/bin/env python
import rospy
import numpy as np
import tf.transformations as tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

class GoToGoal:
    def _init_(self):
        ###--- Inicio del Nodo ---###
        rospy.init_node('go_to_goal')
        rospy.on_shutdown(self.cleanup)

        ###--- Subscriptores ---###
        rospy.Subscriber("/odom", Odometry, self.odom_cb)

        ###--- Publishers ---###
        self.pub_cmd_vel = rospy.Publisher('/puzzlebot_1/base_controller/cmd_vel', Twist, queue_size=1)
        self.pos_t_pub = rospy.Publisher('/target_gtg', PoseStamped, queue_size=1)
        self.at_goal_flag_pub = rospy.Publisher('/finish_path', Bool, queue_size=1)

        ###--- Constants ---###
        self.dt = 0.02
        self.path_points = [(1.0, 1.0), (2.0, 1.0), (3.0, 1.0)]
        self.index = 0
        self.x_target = 1
        self.y_target = 2

        ###--- Objetos ---###
        self.flag_msg = Bool()
        self.pose = PoseStamped()
        self.pose_target = PoseStamped()
        self.v_msg = Twist()
        rate = rospy.Rate(int(1.0/self.dt))

        ###--- Variables ---###
        self.state = "go_to_goal"
        self.at_goal_flag = False
        self.finish_track = False
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.theta_robot = 0.0
        self.clear_path = False

        while rospy.get_time() == 0:
            pass # Espera hasta que el tiempo de ROS esté inicializado

        self.t_ant = rospy.get_time() # Valor para obtener dt

        while not rospy.is_shutdown():
            if self.state == "go_to_goal":
                v, w = self.calc_gtg()
                self.at_goal()
                self.fill_messages(v, w)

                if self.at_goal() and self.index < len(self.path_points) - 1:
                    self.index += 1
                    self.x_target = self.path_points[self.index][0]
                    self.y_target = self.path_points[self.index][1]

                if self.at_goal() and self.index == len(self.path_points) - 1:
                    self.finish_track = True

            self.pub_cmd_vel.publish(self.v_msg)
            self.pos_t_pub.publish(self.pose_target)
            self.at_goal_flag_pub.publish(self.flag_msg)
            rate.sleep()

    def calc_gtg(self):
        x_target = self.x_target
        y_target = self.y_target
        x_robot = self.x_pos
        y_robot = self.y_pos
        theta_robot = self.theta_robot

        kvmax = 0.3  # linear speed maximum gain
        kwmax = 1.0  # angular angular speed maximum gain
        # kw=0.5
        av = 2.0  # Constant to adjust the exponential's growth rate
        aw = 2.0  # Constant to adjust the exponential's growth rate
        ed = np.sqrt((x_target-x_robot)*2+(y_target-y_robot)*2)
        # Compute angle to the target position
        theta_target = np.arctan2(y_target-y_robot, x_target-x_robot)
        e_theta = theta_target-theta_robot
        # limit e_theta from -pi to pi
        # This part is very important to avoid abrupt changes when error switches between 0 and +-2pi
        e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta))
        # Compute the robot's angular speed
        kw = kwmax*(1-np.exp(-aw*e_theta**2)) / \
            abs(e_theta)  # Constant to change the speed
        w = kw*e_theta
        if abs(e_theta) > np.pi/8:
            v = 0  # linear speed
        else:
            kv = kvmax*(1-np.exp(-av*ed**2))/abs(ed)
            v = kv*ed  # linear speed
        return v, w

    def at_goal(self):
        tolerance = 0.1
        x_window = self.x_target - tolerance < self.x_pos < self.x_target + tolerance
        y_window = self.y_target - tolerance < self.y_pos < self.y_target + tolerance
        self.at_goal_flag = x_window and y_window
        return self.at_goal_flag

    def fill_messages(self, v, w):
        self.v_msg.linear.x = v
        self.v_msg.angular.z = w

        self.pose_target.pose.position.x = self.x_target
        self.pose_target.pose.position.y = self.y_target

        quat = quaternion_from_euler(0, 0, self.theta_robot)
        self.pose.pose.orientation.x = quat[0]
        self.pose.pose.orientation.y = quat[1]
        self.pose.pose.orientation.z = quat[2]
        self.pose.pose.orientation.w = quat[3]

        self.flag_msg.data = self.finish_track

    def odom_cb(self, msg):
        self.x_pos = msg.pose.pose.position.x
        self.y_pos = msg.pose.pose.position.y
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        _, _, self.theta_robot = tf.euler_from_quaternion([x, y, z, w])

    def cleanup(self):
        vel_msg = Twist()
        self.pub_cmd_vel.publish(vel_msg)

if __name__ == "__main__":
    GoToGoal()