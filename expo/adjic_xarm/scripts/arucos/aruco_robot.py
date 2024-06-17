#!/usr/bin/env python

import rospy 
import numpy as np
from arucos import aruco_robot as aruco
from fiducial_msgs.msg import FiducialTransformArray
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped
import tf2_ros
from std_msgs.msg import Bool , Float32MultiArray
from xarm_msgs.msg import RobotMsg
from sensor_msgs.msg import CameraInfo
    
class ArucoFinder:
    def __init__(self , mode):
        ###--- Inicio del Nodo ---###
        rospy.init_node('aruco_finder')
        #rospy.wait_for_message("/camera_info" , CameraInfo)
        rospy.on_shutdown(self.cleanup)

        prefix = rospy.get_param("robot" , "")

        ###--- Subscriptores ---###
        rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.ft_cb)
        rospy.Subscriber("/xarm/xarm_states", RobotMsg, self.rb_cb)

        ###--- Publishers ---###
        self.array_pub = rospy.Publisher("/ar_array" , Float32MultiArray , queue_size=1)
        self.flag_pub = rospy.Publisher("/arucos_flag" , Bool , queue_size=1)
        self.tf_pub = tf2_ros.TransformBroadcaster()

        ###---Variables---###
        self.tcp_x = 0
        self.tcp_y = 0
        self.tcp_z = 0
        self.tcp_roll = 0
        self.tcp_pitch = 0
        self.tcp_yaw = 0
    
        ###--- Constants ---###
        self.dt = 0.02

        ###--- Objetos ---####
        self.arucos_1 = aruco(707 , 0.102 , 0 , -0.0025)

        self.flag_msg = Bool()
        self.array_msg = Float32MultiArray()
        t = TransformStamped()
        self.flag = False
        
        self.fiducial_transform = FiducialTransformArray()
        rate = rospy.Rate(int(1.0 / self.dt))

        while rospy.get_time() == 0 or self.flag: pass

        while not rospy.is_shutdown():

            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "base_link"
            t.child_frame_id = "tcp"
            t.transform.translation.x = self.tcp_x
            t.transform.translation.y = self.tcp_y
            t.transform.translation.z = self.tcp_z

            quat = quaternion_from_euler(self.tcp_roll , self.tcp_pitch , self.tcp_yaw)

            t.transform.rotation.x = quat[0]
            t.transform.rotation.y = quat[1]
            t.transform.rotation.z = quat[2]
            t.transform.rotation.w = quat[3]

            self.tf_pub.sendTransform(t)

            t = TransformStamped()

            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "tcp"
            t.child_frame_id = "usb_cam"
            t.transform.translation.x = 0.075
            t.transform.translation.y = 0.015
            t.transform.translation.z = 0.015

            quat = quaternion_from_euler(0,0,np.pi/2)

            t.transform.rotation.x = quat[0]
            t.transform.rotation.y = quat[1]
            t.transform.rotation.z = quat[2]
            t.transform.rotation.w = quat[3]

            self.tf_pub.sendTransform(t)

            flag , ar = self.process_transforms()
            self.pub_msgs(flag , ar)
            rate.sleep()

    def process_transforms(self):
        flag_msg = False
        arr = []
        tras = []
        rot = []

        if self.fiducial_transform.transforms:
            for arucos in self.fiducial_transform.transforms:
                if arucos.fiducial_id == self.arucos_1.ID:
                    flag_msg = True
                    tras_ = arucos.transform.translation
                    rot_ = arucos.transform.rotation
                        
                    self.publish_transforms()

                        
        else:
            flag_msg = False

        return flag_msg , arr
        
    def pub_msgs(self , flag , t):
        self.array_msg.data = t
        self.flag_msg.data = flag

        self.array_pub.publish(self.array_msg)
        self.flag_pub.publish(self.flag_msg)

    def publish_transforms(self):
        t = TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "fiducial_707"
        t.child_frame_id = "pallet_1"
        t.transform.translation.x = 0.032
        t.transform.translation.y = 0.065
        t.transform.translation.z = 0.0

        quat = quaternion_from_euler(0,0,0)

        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_pub.sendTransform(t)

        t = TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "fiducial_707"
        t.child_frame_id = "pallet_2"
        t.transform.translation.x = -0.017
        t.transform.translation.y = 0.065
        t.transform.translation.z = 0.0

        quat = quaternion_from_euler(0,0,0)

        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_pub.sendTransform(t)
    
    def ft_cb(self, msg):
        self.fiducial_transform = msg
        self.flag = True

    def rb_cb(self , msg):
        pose = msg.pose
        self.tcp_x , self.tcp_y , self.tcp_z = pose[0]/1000 , pose[1]/1000 , pose[2]/1000
        self.tcp_roll , self.tcp_pitch , self.tcp_yaw = pose[3] , pose[4] , pose[5]

    def cleanup(self):
        print("Apagando Localización")

if __name__ == "__main__":
    ArucoFinder("real")
