#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode,                           \
                         TrajectorySetpoint,                            \
                         VehicleStatus, VehicleCommand,                 \
                         VehicleGlobalPosition, VehicleLocalPosition,   \
                         VehicleAttitude
from geometry_msgs.msg import Twist, Vector3
from math import pi
import math
from std_msgs.msg import Bool
from .lib  import PathHandler, DummyPlanner, MiddlePointPlanner, \
                LAT, LON, ALT, YAW
import json

class OffboardControl(Node):

    def __init__(self):
        super().__init__('offboard_control')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        #Create subscriptions
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile)
        
        self.offboard_velocity_sub = self.create_subscription(
            Twist,
            '/offboard_velocity_cmd',
            self.offboard_velocity_callback,
            qos_profile)
        
        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_callback,
            qos_profile)

        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self.local_position_callback,
            qos_profile,
        )

        self.global_position_sub = self.create_subscription(
            VehicleGlobalPosition,
            '/fmu/out/vehicle_global_position',
            self.global_position_callback,
            qos_profile)
        
        self.my_arm_bool_sub = self.create_subscription(
            Bool,
            '/arm_message',
            self.arm_message_callback,
            qos_profile)

        self.my_land_bool_sub = self.create_subscription(
            Bool,
            '/land_message',
            self.land_message_callback,
            qos_profile)
        
        self.my_offboard_bool_sub = self.create_subscription(
            Bool,
            '/mission_message',
            self.mission_message_callback,
            qos_profile)
        
        #Create publishers
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_velocity = self.create_publisher(Twist, '/fmu/in/setpoint_velocity/cmd_vel_unstamped', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)

        
        #creates callback function for the arm timer
        # period is arbitrary, just should be more than 2Hz
        arm_timer_period = .1 # seconds
        self.arm_timer_ = self.create_timer(arm_timer_period, self.arm_timer_callback)

        # creates callback function for the command loop
        # period is arbitrary, just should be more than 2Hz. Because live controls rely on this, a higher frequency is recommended
        # commands in cmdloop_callback won't be executed if the vehicle is not in offboard mode
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arm_state = VehicleStatus.ARMING_STATE_ARMED
        self.velocity = Vector3()
        self.cur_waypoint_index = 0
        self.yaw = 0.0  #yaw value we send as command
        self.trueYaw = 0.0  #current yaw value of drone
        self.offboardMode = False
        self.flightCheck = False
        self.myCnt = 0
        self.arm_message = False
        self.land_message = False
        self.mission_message = False
        self.failsafe = False
        self.current_state = "IDLE"
        self.last_state = self.current_state
        self.global_position = np.array([np.nan, np.nan, np.nan, np.nan])
        self.takeoff_position = np.array([np.nan, np.nan, np.nan, np.nan])

        self.declare_parameter('takeoff_altitude', 0.0)

        self.takeoff_altitude = self.get_parameter('takeoff_altitude').value
        # self.declare_parameter('waypoint_list', [])
        # waypoints_str = self.get_parameter('waypoints').get_parameter_value().string_value
        # waypoint_list = json.loads(waypoints_str)  # 문자열을 파이썬 리스트로 변환
        waypoint_list = [
            [37.5478915, 127.1194249, 20.0, float('nan')],
            [37.5474712, 127.1186771, 20.0, float('nan')],
            [37.5468944, 127.1186654, 20.0, float('nan')],
            [37.5467255, 127.1190820, 20.0, float('nan')]
        ] 
        # self.takeoff_altitude = self.get_parameter('altitude').value
        self.waypoints = np.array(waypoint_list)
        self.path = PathHandler(self.waypoints, DummyPlanner).make_path() # DummyPlanner or MiddlePointPlanner

    def arm_message_callback(self, msg):
        self.arm_message = msg.data
        self.get_logger().info(f"Arm Message: {self.arm_message}")

    def land_message_callback(self, msg):
        self.land_message = msg.data
        self.get_logger().info(f"Land Message: {self.land_message}")

    def mission_message_callback(self, msg):
        self.mission_message = msg.data
        self.get_logger().info(f"Mission Message: {self.mission_message}")
    #callback function that arms, takes off, and switches to offboard mode
    #implements a finite state machine
    def arm_timer_callback(self):
        match self.current_state:
            case "IDLE":
                if (self.flightCheck and self.land_message == True):
                    # self.get_logger().info(f"Landing")
                    if (self.arm_state == VehicleStatus.ARMING_STATE_DISARMED):
                        self.land_message = False
                        pass
                elif(self.flightCheck and self.arm_message == True):
                    self.current_state = "ARMING"
                    self.get_logger().info(f"Arming")

            case "ARMING":
                if(not(self.flightCheck)):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Arming, Flight Check Failed")
                elif(self.arm_state == VehicleStatus.ARMING_STATE_ARMED and self.myCnt > 10):
                    self.current_state = "TAKEOFF"
                    self.get_logger().info(f"Arming, Takeoff")
                self.arm() #send arm command

            case "TAKEOFF":
                if(not(self.flightCheck)):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Takeoff, Flight Check Failed")
                elif(self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF):
                    self.current_state = "LOITER"
                    self.get_logger().info(f"Takeoff, Loiter")
                    self.takeoff_position[LAT] = self.global_position[LAT]
                    self.takeoff_position[LON] = self.global_position[LON]
                    self.takeoff_position[ALT] = self.takeoff_altitude
                self.param_MIS_TAKEOFF_ALT() #send takeoff altitude command
                self.arm() #send arm command
                self.take_off() #send takeoff command

            # waits in this state while taking off, and the 
            # moment VehicleStatus switches to Loiter state it will switch to offboard
            case "LOITER": 
                if(not(self.flightCheck)):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Loiter, Flight Check Failed")
                elif(self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER):
                    self.current_state = "OFFBOARD"
                    self.get_logger().info(f"Loiter, Offboard")
                self.arm()

            case "OFFBOARD":
                if(not(self.flightCheck) 
                   or self.arm_state != VehicleStatus.ARMING_STATE_ARMED 
                   or self.failsafe == True):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Offboard, Flight Check Failed")
                elif(self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD 
                     and self.land_message == True):
                    self.current_state = "LAND"
                    self.get_logger().info(f"Land, Idle")
                self.state_offboard()

            case "LAND":
                if (not(self.flightCheck)):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Land, Flight Check Failed")
                elif(self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Land, Idle")
                else:
                    self.land()
                    self.offboardMode = False
                self.get_logger().info(f"Land, Land Message: {self.land_message}")


        if(self.arm_state != VehicleStatus.ARMING_STATE_ARMED):
            self.arm_message = False


        if (self.last_state != self.current_state):
            self.last_state = self.current_state
            self.get_logger().info(self.current_state)
        self.myCnt += 1

    def state_offboard(self):
        param_dict = {
            "param1": 1.0,  # Mode
            "param2": 6.0,  # Mode
        }
        self.myCnt = 0
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param_dict)
        self.offboardMode = True   

    ''' 잘 안됨 '''
    def param_MIS_TAKEOFF_ALT(self):
        param_dict = {
            "param1": 1028.0,  # Parameter index
            "param2": 0.1,  # Parameter value
        }

        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_PARAMETER, param_dict)
        self.get_logger().info("MIS_TAKEOFF_ALT command send")

    # Arms the vehicle
    def arm(self):
        param_dict = { 
          "param1": 1.0   # 0.0 for disarming, 1.0 for ar
        } 

        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param_dict)
        self.get_logger().info("Arm command send")

    def disarm(self):
        param_dict = { 
          "param1": 0.0   # 0.0 for disarming, 1.0 for ar
        } 
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param_dict)
        self.get_logger().info("Disarm command send")

    # Takes off the vehicle to a user specified altitude (meters)
    def take_off(self):
        param_dict = {
            "param1": 1.0,                          # Minimum pitch
            "param4": None,                         # Yaw angle
            "param5": None,                         # Latitude
            "param6": None,                         # Longitude
            "param7": self.takeoff_position[ALT]    # Altitude
        }
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param_dict) # param7 is altitude in meters
        self.get_logger().info("Takeoff command send to " + str(self.takeoff_position[ALT]))

    def land(self):
        param_dict = {
            "param4": None, # Desired yaw angle
            "param5": None, # Latitude
            "param6": None, # Longitude
            "param7": 0.0  # Altitude
        }
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND, param_dict) # VTOL_LAND 안됨
        self.get_logger().info("Land command send")

    def return_to_launch(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)
        self.get_logger().info("RTL command send")

    #publishes command to /fmu/in/vehicle_command
    def publish_vehicle_command(self, command, param: dict ={}):
        msg = VehicleCommand()

        for param_key, param_value in param.items():
            if param_value is None:
                continue
            setattr(msg, param_key, param_value)

        msg.command = command  # command ID
        msg.target_system = 1  # system which should execute the command
        msg.target_component = 1  # component which should execute the command, 0 for all components
        msg.source_system = 1  # system sending the command
        msg.source_component = 1  # component sending the command
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.vehicle_command_publisher_.publish(msg)

    def global_position_callback(self, msg):
        # self.get_logger().info(f"Global Position: {msg.lat} {msg.lon} {msg.alt}")
        self.global_position = np.array([msg.lat, msg.lon, msg.alt, float('nan')])

    def local_position_callback(self, msg):
        # self.get_logger().info(f"Local Position: {msg.x} {msg.y} {msg.z}")
        self.local_position = np.array([msg.x, msg.y, msg.z, float('nan')])



    #receives and sets vehicle status values 
    def vehicle_status_callback(self, msg):

        if (msg.nav_state != self.nav_state):
            self.get_logger().info(f"NAV_STATUS: {msg.nav_state}")
        
        if (msg.arming_state != self.arm_state):
            self.get_logger().info(f"ARM STATUS: {msg.arming_state}")

        if (msg.failsafe != self.failsafe):
            self.get_logger().info(f"FAILSAFE: {msg.failsafe}")
        
        if (msg.pre_flight_checks_pass != self.flightCheck):
            self.get_logger().info(f"FlightCheck: {msg.pre_flight_checks_pass}")

        self.nav_state = msg.nav_state
        self.arm_state = msg.arming_state
        self.failsafe = msg.failsafe
        self.flightCheck = msg.pre_flight_checks_pass

    #receives Twist commands from Teleop and converts NED -> FLU
    def offboard_velocity_callback(self, msg):
        #implements NED -> FLU Transformation
        # X (FLU) is -Y (NED)
        self.velocity.x = -msg.linear.y
        # Y (FLU) is X (NED)
        self.velocity.y = msg.linear.x
        # Z (FLU) is -Z (NED)
        self.velocity.z = -msg.linear.z
        # A conversion for angular z is done in the attitude_callback function(it's the '-' in front of self.trueYaw)
        self.yaw = msg.angular.z

    #receives current trajectory values from drone and grabs the yaw value of the orientation
    def attitude_callback(self, msg):
        orientation_q = msg.q

        #trueYaw is the drones current yaw value
        self.trueYaw = -(np.arctan2(2.0*(orientation_q[3]*orientation_q[0] + orientation_q[1]*orientation_q[2]), 
                                  1.0 - 2.0*(orientation_q[0]*orientation_q[0] + orientation_q[1]*orientation_q[1])))

        
    #publishes offboard control modes and velocity as trajectory setpoints
    def cmdloop_callback(self):
        if (self.offboardMode == False):
            return

        # Publish offboard control modes
        self.offboard_msg_publish()

        if (self.mission_message == False):
            self.hover_msg_publish()
        else:
            self.trajectory_msg_publish()


    def offboard_msg_publish(self):
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position = True 
        offboard_msg.velocity = False 
        offboard_msg.acceleration = False
        self.publisher_offboard_mode.publish(offboard_msg)            

    def hover_msg_publish(self):
        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        trajectory_msg.position[LAT] = self.local_position[LAT]
        trajectory_msg.position[LON] = self.local_position[LON]
        trajectory_msg.position[ALT] = -self.takeoff_position[ALT]
        self.publisher_trajectory.publish(trajectory_msg)

    def trajectory_msg_publish(self):
        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        north_offset, east_offset = \
                               self.calculate_local_offset(self.takeoff_position[LAT], self.takeoff_position[LON], 
                               self.path[self.cur_waypoint_index][LAT], self.path[self.cur_waypoint_index][LON])
        trajectory_msg.position[0] = north_offset
        trajectory_msg.position[1] = east_offset
        trajectory_msg.position[2] = -self.path[self.cur_waypoint_index][ALT]
        # trajectory_msg.velocity[0] = float('nan')
        # trajectory_msg.velocity[1] = float('nan')
        # trajectory_msg.velocity[2] = float('nan')
        # trajectory_msg.acceleration[0] = float('nan')
        # trajectory_msg.acceleration[1] = float('nan')
        # trajectory_msg.acceleration[2] = float('nan')
        # trajectory_msg.yaw = self.yaw
        # trajectory_msg.yawspeed = 0.0
        self.publisher_trajectory.publish(trajectory_msg)
        norm = self.haversine(self.global_position[LAT], self.global_position[LON], self.path[self.cur_waypoint_index][LAT], self.path[self.cur_waypoint_index][LON])
        if norm < 0.1:
            self.cur_waypoint_index += 1 if self.cur_waypoint_index < len(self.path) - 1 else 0
        self.get_logger().info(f"Distance: {norm} [m]")
        self.get_logger().info(f"Waypoint: {self.cur_waypoint_index + 1}")
    import math

    def haversine(self, lat1, lon1, lat2, lon2):
        # 지구 반지름 (미터 단위)
        R = 6371000  
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)

        a = math.sin(delta_phi / 2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        distance = R * c
        return distance  # 결과는 미터 단위

    def calculate_local_offset(self, lat1, lon1, lat2, lon2):
        # 현재 위치에서 북쪽으로 얼마나 떨어져 있는지
        north = self.haversine(lat1, lon1, lat2, lon1)
        # 현재 위치에서 동쪽으로 얼마나 떨어져 있는지
        east = self.haversine(lat1, lon1, lat1, lon2)

        # 남쪽 또는 서쪽으로의 이동에 대한 방향 조정
        if lat2 < lat1:
            north = -north
        if lon2 < lon1:
            east = -east

        return north, east


def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
