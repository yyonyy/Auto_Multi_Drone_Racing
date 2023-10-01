#! /usr/bin/env python3

import rospy
import cv2
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import Point, PoseStamped, Twist, Vector3Stamped, Quaternion
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State, HomePosition, AttitudeTarget, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode, CommandHome, SetModeRequest, CommandBoolRequest
from std_msgs.msg import Bool, Int32, Float32
from math import sin, cos, sqrt, pi
from std_srvs.srv import Empty
import math

class Mission:
    def __init__(self):
        
        self.current_state = State()
        self.global_home_position = GeoPoint()
        self.current_local_position = Point()
        self.home_set = False

        # Home position setting
        self.local_home_position = Point(0, 0, 0) 

        # Publisher
        self.local_pose_pub = rospy.Publisher('/drone2/mavros/setpoint_position/local', PoseStamped, queue_size=10)

        # Subscriber
        rospy.Subscriber('/drone2/mavros/state', State, self.stateCb)
        rospy.Subscriber('/drone2/mavros/home_position/home', HomePosition, self.homeCb)
        rospy.Subscriber('/drone2/mavros/local_position/pose', PoseStamped, self.LocalPositionCb)

        # Service_client
        self.arming_client = rospy.ServiceProxy('/drone2/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/drone2/mavros/set_mode', SetMode)
        self.set_home_client = rospy.ServiceProxy('/drone2/mavros/cmd/set_home', CommandHome)
  
        # Gate Positions & Starting Position
        self.starting_position = Point (-6, 4, 0)
        
        self.gate_positions = [
            (Point(self.starting_position.x, self.starting_position.y, self.starting_position.z + 4.5), -1.57),  # take off
            (Point(-7, -8, 4.5), -2.4), # before gate1
            (Point(-7, -12, 4.5), -1.57), # after gate1
            (Point(-2, -20, 4.5), -1.18), # before gate2
            (Point(2, -20, 4.5), 0), # after gate2
            (Point(7, -28, 4.5), -0.95), # before gate3
            (Point(7, -32, 4.5), -1.57), # after gate3
            (Point(2, -40, 4.5), -2.15), # before gate4
            (Point(-2, -40, 4.5), 3.14), # after gate4
            (Point(-7, -32, 4.5), 2.15), # before gate5
            (Point(-7, -28, 4.5), 1.57), # after gate5
            (Point(-2, -20, 4.5), 0.95), # before gate2 (2)
            (Point(2, -20, 4.5), 0), # after gate2 (2)
            (Point(7, -12, 4.5), 1.18), # before gate6
            (Point(7, -8, 4.5), 1.57), # after gate6
            (Point(2, 0, 4.5), 2.4), # before gate7
            (Point(-2, 0, 4.5), 3.14), # after gate7
        ]
        
    
    def check_FCU_connection(self):
        while not self.current_state.connected:
            rospy.loginfo_throttle(1, "Wait FCU connection")

        rospy.loginfo("FCU connected")

    def check_home_setting(self):
        while not self.home_set:
            rospy.loginfo_throttle(1, "Wait Home position Setting")
        rospy.loginfo("Home position is setted")

    def setMode(self, mode):
        rate = rospy.Rate(0.5)
        while True:
            self.PubLocalPosition(self.local_home_position)

            if self.current_state.mode != mode:
                self.set_mode_client(base_mode=0, custom_mode=mode)
            
            else:
                break

            rate.sleep()
    def stateCb(self, msg):
        prev_state = self.current_state

        self.current_state = msg
        
        if self.current_state.mode != prev_state.mode:
            rospy.loginfo("Current mode: %s" % self.current_state.mode)

        if self.current_state.armed != prev_state.armed:
            rospy.loginfo("Vehicle armed: %r" % self.current_state.armed)

    def homeCb(self, msg):
        self.home_set = True

        self.global_home_position.latitude = msg.geo.latitude
        self.global_home_position.longitude = msg.geo.longitude

    def LocalPositionCb(self, msg):
        self.current_local_position.x = msg.pose.position.x
        self.current_local_position.y = msg.pose.position.y
        self.current_local_position.z = msg.pose.position.z
        
    def PubLocalPosition(self, point, yaw):
        pose = PoseStamped()
                
        # Calculate quaternion
        qw = math.cos(yaw / 2)
        qx = 0
        qy = 0
        qz = math.sin(yaw / 2)
        
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = point.x
        pose.pose.position.y = point.y
        pose.pose.position.z = point.z
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        x = self.current_local_position.x        
        y = self.current_local_position.y
        z = self.current_local_position.z

        d = sqrt((point.x - x)**2 + (point.y - y)**2 + (point.z - z)**2)

        if d < 1:
            self.local_pose_pub.publish(pose)
        
        else:
        
            pose.pose.position.x = x + (point.x - x)/d*1.5
            pose.pose.position.y = y + (point.y - y)/d*1.5
            pose.pose.position.z = z + (point.z - z)/d*1.5

            self.local_pose_pub.publish(pose)

    def waypoint_reach_check(self, wp):

        x = self.current_local_position.x        
        y = self.current_local_position.y
        z = self.current_local_position.z

        d = sqrt((wp.x - x)**2 + (wp.y - y)**2 + (wp.z - z)**2)

        if d < 0.3 :
            return True
        else:
            return False


    def state_callback(self, msg):
        self.current_state = msg

    def fly_through_gates(self):
        offb_set_mode = SetModeRequest()
        offb_set_mode.custom_mode = 'OFFBOARD'

        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True

        last_req = rospy.Time.now()

        # takeoff
        first_gate = self.gate_positions[0]

        while not rospy.is_shutdown():
            if self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                if self.set_mode_client.call(offb_set_mode).mode_sent:
                    rospy.loginfo("OFFBOARD enabled")
                last_req = rospy.Time.now()
            else:
                if not self.current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                    if self.arming_client.call(arm_cmd).success:
                        rospy.loginfo("Vehicle armed")
                    last_req = rospy.Time.now()

            target_position = Point(
                first_gate[0].x - self.starting_position.x,
                first_gate[0].y - self.starting_position.y,
                first_gate[0].z - self.starting_position.z
            )
            self.PubLocalPosition(target_position, first_gate[1])
    
            if self.waypoint_reach_check(target_position):
                break
            rate.sleep()

        rospy.loginfo("Takeoff completed")

        # gates
        while not rospy.is_shutdown():
            for gate_position in self.gate_positions[1:]:
                while not rospy.is_shutdown():
                    if self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                        if self.set_mode_client.call(offb_set_mode).mode_sent:
                            rospy.loginfo("OFFBOARD enabled")
                        last_req = rospy.Time.now()
                    else:
                        if not self.current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                            if self.arming_client.call(arm_cmd).success:
                                rospy.loginfo("Vehicle armed")
                            last_req = rospy.Time.now()

                    target_position = Point(
                        gate_position[0].x - self.starting_position.x,
                        gate_position[0].y - self.starting_position.y,
                        gate_position[0].z - self.starting_position.z
                    )
                    self.PubLocalPosition(target_position, gate_position[1])
            
                    if self.waypoint_reach_check(target_position):
                        break
                    rate.sleep()

            rospy.loginfo("Reached the last point, repeating gate positions")



if __name__ == "__main__":
    rospy.init_node("takeoff_py")

    flight = Mission()

    rate = rospy.Rate(20)

    try:
        flight.check_FCU_connection()
        rospy.sleep(1)
        flight.check_home_setting()
        rospy.sleep(1)

        
        flight.fly_through_gates()

    except rospy.ROSInterruptException:
        pass