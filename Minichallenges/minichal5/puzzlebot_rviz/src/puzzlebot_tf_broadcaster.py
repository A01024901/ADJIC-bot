#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped 
# from visualization_msgs.msg import Marker 
from tf.transformations import quaternion_from_euler 
# Because of transformations 
import tf2_ros
import numpy as np 
from geometry_msgs.msg import TransformStamped 


class PuzzlebotTfClass():  
  def __init__(self):
    rospy.Subscriber("wr", Float32, self.wr_cb)
    rospy.Subscriber("wl", Float32, self.wl_cb)
    rospy.Subscriber("pose_sim", PoseStamped, self.pose_sim_cb) 
    # self.marker_pub = rospy.Publisher("puzzlebot_marker", Marker, queue_size = 1) 
    self.tf_br = tf2_ros.TransformBroadcaster() 
    self.robot_pose = PoseStamped() 
    self.robot_pose.pose.orientation.w = 1.0  # This is necessary to avoid errors with the quaternion. 
    # Variables
    self.wr, self.wl = 0.0, 0.0
    self.theta_r_act, self.theta_r_ant  = 0.0, 0.0
    self.theta_l_act, self.theta_l_ant  = 0.0, 0.0
    self.time_ant, self.time_act = 0.0, 0.0

    rate = rospy.Rate(50)  # The rate of the while loop 

    while not rospy.is_shutdown():
      # Calculate THETA_R and THETA_L 
      self.time_act = rospy.get_time()
      self.theta_r_act = self.theta_r_ant + self.wr * (self.time_act - self.time_ant)
      self.theta_l_act = self.theta_l_ant + self.wl * (self.time_act - self.time_ant)
      self.theta_r_ant, self.theta_l_ant = self.theta_r_act, self.theta_l_act
      self.time_ant = self.time_act
      # Publish TFs to rviz
      self.send_base_link_tf(self.robot_pose) 
      self.send_chassis_link_tf() 
      self.send_wr_link_tf(self.theta_r_act)
      self.send_wl_link_tf(self.theta_l_act)
      ######## Publish a marker to rviz ######### 
      # marker = self.fill_marker(self.robot_pose) 
      # self.marker_pub.publish(marker) 
      rate.sleep() 

  def wr_cb(self, msg): 
    self.wr = msg.data
  
  def wl_cb(self, msg): 
    self.wl = msg.data
  
  def pose_sim_cb(self, msg): 
    self.robot_pose = msg 

  def send_base_link_tf(self, pose_stamped=PoseStamped()): 
    """ This receives the robot's pose and broadcast a transformation. """
    t = TransformStamped() 
    t.header.stamp = rospy.Time.now() 
    t.header.frame_id = "odom" 
    t.child_frame_id = "base_link" 
    #Copy data from the received pose to the tf  
    t.transform.translation.x = pose_stamped.pose.position.x 
    t.transform.translation.y = pose_stamped.pose.position.y 
    t.transform.translation.z = pose_stamped.pose.position.z 
    t.transform.rotation.x = pose_stamped.pose.orientation.x 
    t.transform.rotation.y = pose_stamped.pose.orientation.y 
    t.transform.rotation.z = pose_stamped.pose.orientation.z 
    t.transform.rotation.w = pose_stamped.pose.orientation.w 
    # Send the transformation 
    self.tf_br.sendTransform(t) 

  def send_chassis_link_tf(self): 
    t = TransformStamped() 
    t.header.stamp = rospy.Time.now() 
    t.header.frame_id = "base_link" 
    t.child_frame_id = "chassis" 
    #Copy data from the received pose to the tf  
    t.transform.translation.x = 0.0 
    t.transform.translation.y = 0.0 
    t.transform.translation.z = 0.0 
    quat = quaternion_from_euler(0.0, 0.0, 0.0)  # np.pi/2, 0.0, np.pi/2
    t.transform.rotation.x = quat[0]
    t.transform.rotation.y = quat[1]
    t.transform.rotation.z = quat[2]
    t.transform.rotation.w = quat[3]
    # Send the transformation 
    self.tf_br.sendTransform(t) 

  def send_wr_link_tf(self, theta_r_act):
    t = TransformStamped() 
    t.header.stamp = rospy.Time.now() 
    t.header.frame_id = "base_link" 
    t.child_frame_id = "wheel_right_link" 
    #Copy data from the received pose to the tf  
    t.transform.translation.x = 0.052 
    t.transform.translation.y = 0.098 
    t.transform.translation.z = 0.0 
    quat = quaternion_from_euler(0.0, theta_r_act, 0.0)
    t.transform.rotation.x = quat[0] 
    t.transform.rotation.y = quat[1] 
    t.transform.rotation.z = quat[2] 
    t.transform.rotation.w = quat[3] 
    # Send the transformation 
    self.tf_br.sendTransform(t)

  def send_wl_link_tf(self, theta_l_act):
    t = TransformStamped() 
    t.header.stamp = rospy.Time.now() 
    t.header.frame_id = "base_link"
    t.child_frame_id = "wheel_left_link" 
    #Copy data from the received pose to the tf  
    t.transform.translation.x = 0.052
    t.transform.translation.y = -0.098
    t.transform.translation.z = 0.0 
    quat = quaternion_from_euler(0.0, theta_l_act, 0.0) 
    t.transform.rotation.x = quat[0] 
    t.transform.rotation.y = quat[1] 
    t.transform.rotation.z = quat[2] 
    t.transform.rotation.w = quat[3] 
    # Send the transformation 
    self.tf_br.sendTransform(t) 

  """
  def fill_marker(self, pose_stamped=PoseStamped()): 
    marker = Marker() 
    marker.header.frame_id = "chassis" 
    marker.header.stamp = rospy.Time.now() 
    # set shape, Arrow: 0; Cube: 1; Sphere: 2; Cylinder: 3; Mesh: 10 
    marker.type = 10 
    marker.id = 0 
    # Use the stl file chassis.stl,
    marker.mesh_resource = "package://puzzlebot_rviz/meshes/chassis.stl" 
    # Set the scale of the marker 
    marker.scale.x = 1 
    marker.scale.y = 1 
    marker.scale.z = 1 
    # Set the color 
    marker.color.r = 1.0 
    marker.color.g = 0.0 
    marker.color.b = 0.0 
    marker.color.a = 1.0 
    # Set the pose of the marker 
    marker.pose.position.x = 0.0 
    marker.pose.position.y = 0.0 
    marker.pose.position.z = 0.0 
    # Set the marker orientation 
    marker.pose.orientation.x = 0.0 
    marker.pose.orientation.y = 0.0 
    marker.pose.orientation.z = 0.0 
    marker.pose.orientation.w = 1.0 
    return marker 
  """


############################### MAIN PROGRAM ####################################  
if __name__ == "__main__":  
  # first thing, init a node! 
  rospy.init_node('puzzlebot_tf_broadcaster')  
  PuzzlebotTfClass()  
  