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
    def __init__(self):  
        ###--- Inicio del Nodo ---###
        rospy.init_node('go_to_goal')
        rospy.on_shutdown(self.cleanup)

        ###--- Subscriptores ---###
        rospy.Subscriber("odom", Odometry, self.odom_cb)  
        rospy.Subscriber("front_clear", Bool, self.front_clear_cb)

        ###--- Publishers ---###
        self.pub_cmd_vel = rospy.Publisher('gtg_twist', Twist, queue_size=1)  
        self.pos_pub = rospy.Publisher('pos_gtg', PoseStamped, queue_size=1)
        self.pos_t_pub = rospy.Publisher('target_gtg', PoseStamped, queue_size=1)
        self.at_goal_flag_pub = rospy.Publisher('at_goal_flag', Bool, queue_size=1)

        ###--- Constants ---###
        self.dt = 0.02
        map = int(rospy.get_param('world_number' , "1"))
        print (map)
        #positions = [[0.75 , -0.5] , [7.25 , -2.7] , [4.5,-2.25] , [4.5,-2.25]]
        positions = [[0.75 , -0.5] , [4.25 , -2.7] , [4.5,-2.25] , [4.5,-2.25]]
        #positions = [[1.0 , -0.5] , [0.10 , -3.45] , [4.75,-2.25] , [4.75,-2.25]]
        self.x_target = positions[map][0]
        self.y_target = positions[map][1]

        ###--- Objetos ---###
        self.flag_msg = Bool()
        self.pose = PoseStamped()
        self.pose_target = PoseStamped()
        self.v_msg = Twist()
        rate = rospy.Rate(int(1.0/self.dt))

        ###--- Variables ---###
        self.state = "go_to_goal"
        self.at_goal_flag = False
        self.x_pos = 0.0 
        self.y_pos = 0.0
        self.theta_robot = 0.0
        self.clear_path = False

        while rospy.get_time() == 0: pass #Descomentar en simulacion 

        self.t_ant = rospy.get_time() #Valor para obtener dt 

        while not rospy.is_shutdown(): 
            if self.state == "go_to_goal":
                v, w = self.calc_gtg()
                self.at_goal()
                self.fill_messages(v, w)
            elif self.state == "follow_walls":
                v, w = self.follow_walls_behave()

            self.check_state_transition()

            self.pub_cmd_vel.publish(self.v_msg) 
            self.pos_pub.publish(self.pose)
            self.pos_t_pub.publish(self.pose_target)
            self.at_goal_flag_pub.publish(self.flag_msg)
            rate.sleep() 

    def calc_gtg (self):
        kvmax = 0.17
        kwmax = 0.8
        av = 2.0
        aw = 2.0
        ed = np.sqrt((self.x_target - self.x_pos)**2+(self.y_target - self.y_pos)**2)
        theta_target = np.arctan2(self.y_target - self.y_pos , self.x_target - self.x_pos)
        e_theta = theta_target - self.theta_robot
        e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta))

        if e_theta != 0:
            kw = kwmax * (1 - np.exp(-aw * e_theta **2))/abs(e_theta)
        else:
            kw = 0.05
        w = kw * e_theta
        if abs(e_theta) > np.pi/8:
            v = 0
        else:
            kv = kvmax * (1 - np.exp(-av * ed **2))/abs(ed)
            v = kv * ed

        return v, w
    
    def follow_walls_behave(self):
        # Aquí implementa la lógica para seguir las paredes
        v = 0.0
        w = 0.0
        return v, w

    def check_state_transition(self):
        if self.state == "go_to_goal" and self.at_goal_flag:
            self.state = "follow_walls"
        elif self.state == "follow_walls" and self.clear_path:
            self.state = "go_to_goal"

    def at_goal(self):
        tolerance = 0.1
        x_window = self.x_target - tolerance < self.x_pos < self.x_target + tolerance
        y_window = self.y_target - tolerance < self.y_pos < self.y_target + tolerance 
        self.at_goal_flag = x_window and y_window

    def fill_messages(self, v, w):
        self.v_msg.linear.x = v
        self.v_msg.angular.z = w
        self.pose.pose.position.x = self.x_pos
        self.pose.pose.position.y = self.y_pos
        self.pose_target.pose.position.x = self.x_target
        self.pose_target.pose.position.y = self.y_target

        quat = quaternion_from_euler(0 , 0 , self.theta_robot)
        self.pose.pose.orientation.x = quat[0]
        self.pose.pose.orientation.y = quat[1]
        self.pose.pose.orientation.z = quat[2]
        self.pose.pose.orientation.w = quat[3]

        self.flag_msg.data = self.at_goal_flag

    def odom_cb(self, msg):
        self.x_pos = msg.pose.pose.position.x
        self.y_pos = msg.pose.pose.position.y
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        _ , _ , self.theta_robot = tf.euler_from_quaternion([x , y , z ,w])

    def front_clear_cb(self, msg):
        self.clear_path = msg.data

    def cleanup(self):  
        vel_msg = Twist() 
        self.pub_cmd_vel.publish(vel_msg) 

if __name__ == "__main__":  
    GoToGoal()

