#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode,                           \
                         TrajectorySetpoint, TrajectoryWaypoint,        \
                         VehicleTrajectoryWaypoint,                     \
                         VehicleTrajectoryBezier,                       \
                         VehicleStatus, VehicleCommand,                 \
                         VehicleGlobalPosition, VehicleLocalPosition,   \
                         VehicleAttitude,                               \
                         PositionSetpointTriplet
from geometry_msgs.msg import Twist, Vector3
from math import pi
import math
from std_msgs.msg import Bool
from .lib  import PathHandler, DummyPlanner, MiddlePointPlanner,    \
                LAT, LON, ALT, YAW,                                 \
                DICT_NAVIGATION_STATE, DICT_VEHICLE_TYPE
import json
import math



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

        self.position_setpoint_triplet_sub = self.create_subscription(
            PositionSetpointTriplet,
            '/fmu/out/position_setpoint_triplet',
            self.position_setpoint_triple_callback,
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
        self.publisher_offboard_mode \
            = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_velocity \
            = self.create_publisher(Twist, '/fmu/in/setpoint_velocity/cmd_vel_unstamped', qos_profile)
        self.publisher_trajectory \
            = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.publisher_vehicle_trajectory_waypoint \
            = self.create_publisher(VehicleTrajectoryWaypoint, '/fmu/in/vehicle_trajectory_waypoint', qos_profile)
        self.publisher_vehicle_trajectory_bezier \
            = self.create_publisher(VehicleTrajectoryBezier, '/fmu/in/vehicle_trajectory_bezier', qos_profile)
        self.publisher_vehicle_command \
            = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)

        
        #creates callback function for the arm timer
        # period is arbitrary, just should be more than 2Hz
        arm_timer_period = .1 # seconds
        self.arm_timer_ = self.create_timer(arm_timer_period, self.arm_timer_callback)

        # creates callback function for the command loop
        # period is arbitrary, just should be more than 2Hz. Because live controls rely on this, a higher frequency is recommended
        # commands in cmdloop_callback won't be executed if the vehicle is not in offboard mode
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)

        self.current_state = "IDLE"
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_ARMED
        self.vehicle_type = VehicleStatus.VEHICLE_TYPE_UNKNOWN
        self.in_transition_mode = False
        self.in_transition_to_fw = False
        self.velocity = Vector3()
        self.cur_waypoint_index = 0
        self.mission_progress = 0
        ''' < Mission Progress >
        0 : IDLE 상태,                 | 이륙 수행
        1 : 이륙 성공,                 | VTOL 천이(MC → FW) 수행
        2 : VTOL 천이(MC → FW) 성공,   | FW Trajectory Mission 수행
        3 : FW Trajectory Mission 성공,| VTOL 천이(FW → MC) 수행
        4 : VTOL 천이(FW → MC) 성공,   | MC 구조 Mission 수행
        5 : MC 구조 Mission 수행,      | Land 상태로 진입
        '''
        self.yaw                    = 0.0  #yaw value we send as command
        self.trueYaw                = 0.0  #current yaw value of drone
        self.takeoff_time           = None        # microseconds
        self.armed_time             = None        # microseconds
        self.offboardMode           = False
        self.pre_flight_checks_pass = False
        self.my_cnt                 = 0
        self.my_mission_cnt         = -1
        self.arm_message            = False
        self.land_message           = False
        self.mission_message        = False
        self.failsafe               = False
        self.last_state             = self.current_state
        self.global_position        = np.array([float('nan'), float('nan'), float('nan'), float('nan')])
        self.takeoff_position       = np.array([float('nan'), float('nan'), float('nan'), float('nan')])
        self.waypoint_sent          = False     # waypoint가 전송되었는지 여부

        self.declare_parameter('takeoff_altitude', 5.0)

        self.takeoff_altitude = self.get_parameter('takeoff_altitude').value
        # self.declare_parameter('waypoint_list', [])
        # waypoints_str = self.get_parameter('waypoints').get_parameter_value().string_value
        # waypoint_list = json.loads(waypoints_str)  # 문자열을 파이썬 리스트로 변환
        waypoint_list = [
            # [37.5478915, 127.1194249, 20.0, float('nan')],
            # [37.5474712, 127.1186771, 20.0, float('nan')],
            # [37.5468944, 127.1186654, 20.0, float('nan')],
            # [37.5467255, 127.1190820, 20.0, float('nan')],
            [37.000020299999996, 24.0000292, 20, float('nan')],
            [37.0038957, 24.0000292, 20, float('nan')],
            [37.000020299999996, 24.0000292, 20, float('nan')],
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
                if (self.pre_flight_checks_pass and self.land_message == True):
                    # self.get_logger().info(f"Landing")
                    if (self.arming_state == VehicleStatus.ARMING_STATE_DISARMED):
                        self.land_message = False
                        pass
                elif(self.pre_flight_checks_pass and self.arm_message == True):
                    self.current_state = "ARMING"
                    self.get_logger().info(f"Arming")

            case "ARMING":
                if(not(self.pre_flight_checks_pass)):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Arming, Flight Check Failed")
                elif(self.arming_state == VehicleStatus.ARMING_STATE_ARMED and self.my_cnt > 10):
                    self.current_state = "TAKEOFF"
                    self.get_logger().info(f"Arming, Takeoff")
                self.arm() #send arm command

            case "TAKEOFF":
                if(not(self.pre_flight_checks_pass)):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Takeoff, Flight Check Failed")
                elif(self.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF):
                    self.get_logger().info(f"Takeoff, Loiter")
                    self.takeoff_position[LAT] = self.global_position[LAT]
                    self.takeoff_position[LON] = self.global_position[LON]
                    self.takeoff_position[ALT] = self.takeoff_altitude
                    self.take_off() #send takeoff command
                elif(self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF):
                    self.get_logger().info(f"Takeoff Success")
                    self.current_state = "LOITER"
                    self.mission_progress = 1
                # if (self.armed_time is not None and self.takeoff_time is not None
                #       and self.global_position[ALT] > self.takeoff_position[ALT] * 0.7):
                #     self.current_state = "LOITER"
                #     self.get_logger().info(f"Takeoff Success")
                # self.param_MIS_TAKEOFF_ALT() #send takeoff altitude command
                self.arm() #send arm command

            # waits in this state while taking off, and the 
            # moment VehicleStatus switches to Loiter state it will switch to offboard
            case "LOITER": 
                if(not(self.pre_flight_checks_pass)):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Loiter, Flight Check Failed")
                elif((self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER
                      or self.nav_state == VehicleStatus.NAVIGATION_STATE_POSCTL)
                     and self.mission_progress != 0):
                     match self.mission_progress:
                        case 1:
                            self.current_state = "TRANSITION_FW"
                        case 2:
                            pass
                        case 3:
                            self.current_state = "TRANSITION_MC"
                        case 4:
                            # pass
                            self.state_hold()
            case "TRANSITION_FW":
                if(not(self.pre_flight_checks_pass)):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Transition, Flight Check Failed")
                elif(self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER):
                    self.transition_MC_to_FW()
                    self.get_logger().info(f"Transition, Offboard")
                    if(self.in_transition_mode == True):# and self.in_transition_to_fw == True):
                        self.mission_progress = 2
                    elif(self.mission_progress == 2 and self.in_transition_mode == False):#and self.in_transition_to_fw == False):
                        self.current_state = "POSCTL"
                        self.get_logger().info(f"VTOL transition complete (MC → FW). Offboard mode enabled.")

            case "POSCTL":
                if(not(self.pre_flight_checks_pass)):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Position Control, Flight Check Failed")
                elif(self.nav_state == VehicleStatus.NAVIGATION_STATE_POSCTL and
                     self.mission_progress == 2 and self.my_mission_cnt == -1):
                    self.my_mission_cnt = self.my_cnt
                    self.get_logger().info(f"Position Control, Offboard")
                elif(self.nav_state == VehicleStatus.NAVIGATION_STATE_POSCTL and
                     self.my_cnt > self.my_mission_cnt + 30):
                    # self.state_hold()
                    self.current_state = "LOITER"
                    self.mission_progress = 3
                self.get_logger().info(f"mission_progress: {self.mission_progress}, my_cnt: {self.my_cnt}, my_mission_cnt: {self.my_mission_cnt}")
                self.state_posctl()

            case "OFFBOARD":
                if(not(self.pre_flight_checks_pass) 
                   or self.arming_state != VehicleStatus.ARMING_STATE_ARMED 
                   or self.failsafe == True):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Offboard, Flight Check Failed")
                elif(self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD 
                     and self.land_message == True):
                    self.current_state = "LAND"
                    self.get_logger().info(f"Land, Idle")
                elif (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
                    and self.mission_progress == 3):
                    self.current_state = "LOITER"
                self.state_offboard()
                self.offboardMode = True   

            case "TRANSITION_MC":
                if (not(self.pre_flight_checks_pass)):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Transition, Flight Check Failed")
                else:
                    self.transition_FW_to_MC()
                    if(self.in_transition_mode == True):# and self.in_transition_to_fw == True):
                        self.mission_progress = 4
                    elif(self.mission_progress == 4 and self.in_transition_mode == False):
                        self.current_state = "LOITER"
                        self.get_logger().info(f"VTOL transition complete (MC → FW). Offboard mode enabled.")
                    self.get_logger().info(f"VTOL transition complete (MC → FW). Offboard mode enabled.")

            case "LAND":
                if (not(self.pre_flight_checks_pass)):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Land, Flight Check Failed")
                elif(self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Land, Idle")
                else:
                    self.land()
                    self.offboardMode = False
                self.get_logger().info(f"Land, Land Message: {self.land_message}")


        if(self.arming_state != VehicleStatus.ARMING_STATE_ARMED):
            self.arm_message = False


        if (self.last_state != self.current_state):
            self.last_state = self.current_state
            self.get_logger().info(self.current_state)
        self.my_cnt += 1


    def transition_MC_to_FW(self):
        param_dict = {
            "param1": 4.0,  # Mode
        }

        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_VTOL_TRANSITION, param_dict)
        self.get_logger().info("Transition(MC → FW) command send")

    def transition_FW_to_MC(self):
        param_dict = {
            "param1": 3.0,  # Mode
        }

        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_VTOL_TRANSITION, param_dict)
        self.get_logger().info("Transition(FW → MC) command send")

    def state_offboard(self):
        param_dict = {
            "param1": 1.0,  # Mode
            "param2": 6.0,  # Mode
        }
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param_dict)

    def state_posctl(self):
        ''' Position Control Mode '''
        param_dict = {
            "param1": 1.0,  # Mode
            "param2": 3.0,  # Mode
        }
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param_dict)
        self.get_logger().info("Position Control Mode command send")

    def state_hold(self):
        ''' Position Control Mode '''
        param_dict = {
            "param1": 1.0,  # Mode
            "param2": 4.0,  # Mode
            "param3": 3.0,  # Mode
        }
        # 1 : Manual
        # 2 : Altitude
        # 3 : Position
        # 4 : Mission
        # 5 : Acro
        # 6 : Offboard
        # 7 : Stabilized
        # 8 : Rattitude
        # 9 : Auto
        # 10 : Guided
        # 11 : Hold


        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param_dict)
        self.get_logger().info("Hold Control Mode command send")


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
            "param5": self.takeoff_position[LAT],   # Latitude
            "param6": self.takeoff_position[LON],   # Longitude
            "param7": self.takeoff_position[ALT]    # Altitude
        }
        self.get_logger().info(f"Takeoff Position: {self.takeoff_position[LAT]}, {self.takeoff_position[LON]}, {self.takeoff_position[ALT]}")
        # self.get_logger().info("Takeoff command send to " + str(self.takeoff_position[ALT]))
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param_dict) # param7 is altitude in meters

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
        self.publisher_vehicle_command.publish(msg)

    def global_position_callback(self, msg):
        # self.get_logger().info(f"Global Position: {msg.lat} {msg.lon} {msg.alt}")
        self.global_position = np.array([msg.lat, msg.lon, msg.alt, float('nan')])

    def local_position_callback(self, msg):
        # self.get_logger().info(f"Local Position: {msg.x} {msg.y} {msg.z}")
        self.local_position = np.array([msg.x, msg.y, msg.z, float('nan')])


    def position_setpoint_triple_callback(self, msg):
        pass
        # self.get_logger().info(f"Position Setpoint Triplet: {msg.current.lat}")
        # self.get_logger().info(f"Position Setpoint Triplet: {msg.current.lon}")
        # self.get_logger().info(f"Position Setpoint Triplet: {msg.current.alt}")

    #receives and sets vehicle status values 
    def vehicle_status_callback(self, msg):

        # if (msg.nav_state != self.nav_state):
        self.get_logger().info(f"")
        self.get_logger().info(f"NAV_STATUS          : {DICT_NAVIGATION_STATE[msg.nav_state]}")
        self.get_logger().info(f"Current State       : {self.current_state}")
        
        if (msg.arming_state != self.arming_state):
            self.get_logger().info(f"ARM STATUS: {msg.arming_state}")

        if (msg.failsafe != self.failsafe):
            self.get_logger().info(f"FAILSAFE: {msg.failsafe}")
        
        if (msg.pre_flight_checks_pass != self.pre_flight_checks_pass):
            self.get_logger().info(f"FlightCheck: {msg.pre_flight_checks_pass}")

        if (self.armed_time != msg.armed_time):
            self.get_logger().info(f"Armed Time: {msg.armed_time}")
        if (self.takeoff_time != msg.takeoff_time):
            self.get_logger().info(f"Takeoff Time: {msg.takeoff_time}")

        self.get_logger().info(f"Mission Progress    : {self.mission_progress}")
        self.get_logger().info(f"Vehicle Type        : {DICT_VEHICLE_TYPE[msg.vehicle_type]}")
        self.get_logger().info(f"VTOL trasintion mode: {msg.in_transition_mode}")
        self.get_logger().info(f"VTOL MC -> FW       : {msg.in_transition_to_fw}")

        self.armed_time = msg.armed_time
        self.takeoff_time = msg.takeoff_time
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state
        self.vehicle_type = msg.vehicle_type
        self.in_transition_mode = msg.in_transition_mode
        self.in_transition_to_fw = msg.in_transition_to_fw
        self.failsafe = msg.failsafe
        self.pre_flight_checks_pass = msg.pre_flight_checks_pass


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
        if (self.mission_progress % 2 != 0):
            return

        self.offboard_msg_publish()
        # if (self.mission_message == False):
        # self.hover_msg_publish()
        # else:
        # if self.waypoint_sent == False:
        # self.trajectory_msg_publish()
        # self.vehcile_trajectory_waypoint_msg_publish()

    def offboard_msg_publish(self):
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position = True 
        offboard_msg.velocity = False 
        offboard_msg.acceleration = False
        offboard_msg.body_rate = False
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
        # self.get_logger().info(f"Distance: {norm} [m]")
        # self.get_logger().info(f"Waypoint: {self.cur_waypoint_index + 1}")


    def vehcile_trajectory_waypoint_msg_publish(self):
        msg = VehicleTrajectoryWaypoint()
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        msg.type = VehicleTrajectoryWaypoint.MAV_TRAJECTORY_REPRESENTATION_WAYPOINTS
        self.get_logger().info(f"Vehicle Trajectory Waypoint: {len(self.path)} waypoints")
        for i in range(len(self.path)):
            wp = TrajectoryWaypoint()
            north_offset, east_offset = \
                               self.calculate_local_offset(self.takeoff_position[LAT], self.takeoff_position[LON], 
                               self.path[self.cur_waypoint_index][LAT], self.path[self.cur_waypoint_index][LON])
            wp.position = [ north_offset, east_offset, -self.path[i][ALT]],  # 고도 음수로 (ENU 기준) 
            # wp.velocity = [0, 0, 0 ]  # 속도는 NaN으로 설정
            # wp.acceleration = [float('nan'), float('nan'), float('nan')]

            msg.waypoints[i] = wp
            # msg.waypoints.yaw = self.path[i][YAW]
        self.publisher_vehicle_trajectory_waypoint.publish(msg)




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
