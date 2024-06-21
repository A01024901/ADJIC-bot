#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
from xarm_msgs.srv import Move, MoveRequest, SetInt16, Call , SetAxis , TCPOffset , ClearErr
from tf.transformations import euler_from_quaternion
import movements as mv
import tf2_ros

class Xarm_place(): 
    def __init__(self , pallets_ini , observation_ini , home_ini , z_offset): 
        ###--- Init node ---###
        rospy.on_shutdown(self.cleanup)
        self.prefix = rospy.get_param("/robot" , "xarm")
        print(self.prefix)

        ###--- Wait for services ---###
        rospy.wait_for_service("/" + self.prefix + '/set_tcp_offset')
        rospy.wait_for_service("/" + self.prefix + '/move_joint')
        rospy.wait_for_service("/" + self.prefix + '/move_line')
        rospy.wait_for_service("/" + self.prefix + '/open_lite6_gripper')
        rospy.wait_for_service("/" + self.prefix + '/close_lite6_gripper')
        
        ###--- Init services ---###
        self.init_services()

        ###--- Tf listener ---###
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)

        ###--- Subscriber ---###
        rospy.Subscriber("/at_place", Bool , self.at_place_cb)
        rospy.Subscriber("/at_pick" , Bool , self.at_pick_cb)

        ###--- Publisher ---###
        self.ready_pub = rospy.Publisher("/" + self.prefix + "/cobot_ready", Bool , queue_size=1)
        
        ###--- Motion services ---###
        self.move_joint_service_proxy = rospy.ServiceProxy("/" + self.prefix + '/move_joint', Move)
        self.move_line_service_proxy = rospy.ServiceProxy("/" + self.prefix + '/move_line', Move)
        self.open_gripper_service_proxy = rospy.ServiceProxy("/" + self.prefix + '/open_lite6_gripper', Call)
        self.close_gripper_service_proxy = rospy.ServiceProxy("/" + self.prefix + '/close_lite6_gripper', Call)

        ###--- Parameters ---###
        self.z_offset = z_offset
        self.velocity = 0.35  # rad/s para joint
        self.acceleration = 6.5  # rad/s^2 para joint
        self.line_velocity = 150  # mm/s para lineal
        self.line_acceleration = 1500  # mm/s^2 para lineal

        ###--- Variables ---###
        rate = rospy.Rate(20)
        self.msg = Bool()
        self.msg.data = False
        self.at_pick = False
        self.at_place = False
        pallets = pallets_ini
        home = home_ini
        observation = observation_ini

        ###--- Move home ---###
        self.move_to_pose(observation["type"] , observation["pose"])
        rospy.sleep(2)
        self.execute_place()
        self.move_to_pose(observation["type"] , observation["pose"])
        rospy.sleep(2)

        while not rospy.is_shutdown():
            self.msg.data = False
            if self.execute_condition():
                self.msg.data = self.execute(pallets , home , observation)
                print("Pallets")

            else:
                #print("Puzzlebot not ready")
                self.msg.data = False
                pass

            print(self.msg.data)
            self.ready_pub.publish(self.msg)
            rate.sleep()

    def execute_condition(self):
        return self.at_place

    def execute(self , pallets , home , observation):
        try:
            #rospy.sleep(5)
            pallets_r , aruco = self.get_pallets()
            for pallet_r , pallet in zip(pallets_r , pallets):
                self.control_gripper("open")
                rospy.sleep(1)

                print("To pallet")
                self.execute_sequence(pallet)

                print("Pallet")
                self.move_to_pose(pallet_r["type"] , pallet_r["pose"])
                #rospy.sleep(3)

                self.control_gripper("open")
                rospy.sleep(3)

                print("Home")
                self.move_to_pose(home["type"] , home["pose"])
                #rospy.sleep(3)
                    
            print("Observation")
            self.move_to_pose(observation["type"] , observation["pose"])
            #rospy.sleep(8)
                    
            self.at_place = False
            return True

        except:
            print("Robot no iniciado")
            return None
        
    def execute_place(self):
        try:
            self.execute_sequence(mv.pl_full)

        except:
            print("Robot no iniciado")

    def move_to_pose(self, move_type, pose):
        move_request = MoveRequest()
        move_request.pose = pose
        if move_type == 'joint':
            move_request.mvvelo = self.velocity
            move_request.mvacc = self.acceleration
            try:
                response = self.move_joint_service_proxy(move_request)
            except rospy.ServiceException as e:
                print(f"Error al llamar al servicio move_joint: {e}")

        elif move_type == 'linear':
            move_request.mvvelo = self.line_velocity
            move_request.mvacc = self.line_acceleration
            try:
                response = self.move_line_service_proxy(move_request)
            except rospy.ServiceException as e:
                print(f"Error al llamar al servicio move_line: {e}")

    def control_gripper(self, action):
        try:
            if action == "open":
                response = self.open_gripper_service_proxy()
            elif action == "close":
                response = self.close_gripper_service_proxy()
            elif action == 'NA': pass
        except rospy.ServiceException as e:
            print(f"Error al llamar al servicio gripper: {e}")

    def execute_sequence(self , movements):
        for i, movement in enumerate(movements):
            self.move_to_pose(movement['type'], movement['pose'])
            #rospy.sleep(4)
            self.control_gripper(movement['gripper'])
            rospy.sleep(3)

    def init_services(self):
        motion_ctl = rospy.ServiceProxy("/" + self.prefix + '/motion_ctrl', SetAxis)
        set_mode = rospy.ServiceProxy("/" + self.prefix + '/set_mode', SetInt16)
        set_state = rospy.ServiceProxy("/" + self.prefix + '/set_state', SetInt16)
        clear_err = rospy.ServiceProxy("/" + self.prefix + '/clear_err', ClearErr)
        rospy.set_param("/" + self.prefix + '/wait_for_finish' , True)
        rospy.set_param("/" + self.prefix + '/wait_for_finish' , True)

        try:
            clear_err()
            motion_ctl(8 , 1)
            set_mode(0)
            set_state(0)

        except rospy.ServiceException as e:
            print("Service failed " , e)

    def get_pallets(self):
        base_link = self.prefix + '/base_link'
        pallet_1 = self.prefix + '/pallet_1'
        pallet_2 = self.prefix + '/pallet_2'
        try: 
            p1 = self.buffer.lookup_transform(base_link , pallet_1 , rospy.Time())
            t1 = p1.transform
            p2 = self.buffer.lookup_transform(base_link , pallet_2 , rospy.Time())
            t2 = p2.transform
            aruco = self.buffer.lookup_transform(base_link ,'fiducial_707' , rospy.Time())
            aruco_t = aruco.transform

            or_1 = self.get_rpy(t1.rotation)
            or_2 = self.get_rpy(t2.rotation)
            aruco_or = self.get_rpy(aruco_t.rotation)

            m1 = {'type': 'linear', 'pose': [t1.translation.x * 1000, t1.translation.y * 1000, self.z_offset, 3.1416, 0.0, or_1[2]+1.578]}
            m2 = {'type': 'linear', 'pose': [t2.translation.x * 1000, t2.translation.y * 1000, self.z_offset, 3.1416, 0.0, or_2[2]+1.578]}
            m3 = {'type': 'linear', 'pose': [aruco_t.translation.x * 1000, aruco_t.translation.y * 1000, self.z_offset, 3.1416, 0.0, aruco_or[2]+1.578]}

            return [m2] , m3

        except:
            print("Not found")

            return None
        
    def get_palletsV2(self):
        p1_vector = []
        p2_vector = []
        try: 
            for i in range(10):
                p1_vector.append(self.buffer.lookup_transform("base_link" ,'pallet_1' , rospy.Time()).transfrom)
                p2_vector.append(self.buffer.lookup_transform("base_link" ,'pallet_2' , rospy.Time()).transfrom)
                rospy.sleep(0.1)

            #for j in p1_vector:
                #p1_translation = 
            
            #aruco = self.buffer.lookup_transform("base_link" ,'fiducial_707' , rospy.Time())
            #aruco_t = aruco.transform

            #or_1 = self.get_rpy(t1.rotation)
            #or_2 = self.get_rpy(t2.rotation)
            #aruco_or = self.get_rpy(aruco_t.rotation)

            #m1 = {'type': 'linear', 'pose': [t1.translation.x * 1000, t1.translation.y * 1000, 218.6, 3.1416, 0.0, or_1[2]+1.578]}
            #m2 = {'type': 'linear', 'pose': [t2.translation.x * 1000, t2.translation.y * 1000, 218.6, 3.1416, 0.0, or_2[2]+1.578]}
            #m3 = {'type': 'linear', 'pose': [aruco_t.translation.x * 1000, aruco_t.translation.y * 1000, 216.6, 3.1416, 0.0, aruco_or[2]+1.578]}

            #return [m1 , m2] , m3

        except:
            print("Not found")

            return None
        
    def get_rpy(self , rot):
        roll , pitch , yaw = euler_from_quaternion([rot.x , rot.y , rot.z , rot.w])
        return [roll , pitch , yaw]
 
    def at_place_cb(self , msg):
        self.at_place = msg.data

    def at_pick_cb(self , msg):
        self.at_pick = msg.data

    def cleanup(self):
        print("Cobot_dead")

############################### PROGRAMA PRINCIPAL ####################################
if __name__ == "__main__": 
    rospy.init_node("place_xarm") 
    Xarm_place(mv.pl_pallets , mv.pl_observation , mv.pl_home , 218.6)