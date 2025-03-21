#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2022 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

__author__ = "Jaeyoung Lim"
__contact__ = "jalim@ethz.ch"

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleGlobalPosition
from nav_msgs.msg import Path



class OffboardControl(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.declare_parameter("path_clearing_timeout", 5.0)

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',  # fmu in or out => communication  fmu/out : px4 --> ros2 fmu/in : ros2 --> px4
            self.vehicle_status_callback,
            qos_profile)
        
        self.global_position_sub = self.create_subscription(
            VehicleGlobalPosition,
            "/fmu/out/vehicle_global_position",
            self.vehicle_global_position_callback,
            qos_profile,
        )
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_vehicle_command = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
        self.dt = timer_period
        self.declare_waypoint = np.array([0,0, 0,0, 0,0])
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.vehicle_global_position = np.array([0.0, 0.0, 0.0])
        self.state = "INIT"
        self.last_local_pos_update = 0.0
        self.vehicle_path_msg = Path()  # 경로 메시지 추가
    def vehicle_global_position_callback(self, msg):

        # TODO: handle NED->ENU transformation
        self.vehicle_global_position[0] = msg.lat
        self.vehicle_global_position[1] = msg.lon
        self.vehicle_global_position[2] = msg.alt
        # print("vehicle_global_position")
    def vehicle_status_callback(self, msg):
        # TODO: handle NED->ENU transformation
        print("NAV_STATUS: ", msg.nav_state)
        print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def cmdloop_callback(self):
        # # Publish offboard control modes
        # if (self.state == "INIT" and self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD):
        #     # Switch to offboard mode
        #     self.send_vehicle_command = VehicleCommand()
        #     self.send_vehicle_command.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        #     self.send_vehicle_command.param1 = 14.0
        #     self.state = "OFFBOARD"
        #     self.get_logger().info("Switching to OFFBOARD mode.")
        #     self.publisher_vehicle_command.publish(self.send_vehicle_command)
        # offboard_msg = OffboardControlMode()
        # offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        # offboard_msg.position=True
        # offboard_msg.velocity=False
        # offboard_msg.acceleration=False
        # self.publisher_offboard_mode.publish(offboard_msg)
        

        # if (self.state == "OFFBOARD" and self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.arming_state != VehicleStatus.ARMING_STATE_ARMED):
        #     # Arm the vehicle   
        #     self.send_vehicle_command = VehicleCommand()
        #     self.send_vehicle_command.command = 400 
        #     self.send_vehicle_command.param2 = None
        #     self.send_vehicle_command.param1 = 1.0
        #     self.send_vehicle_command.param3 = None
        #     self.send_vehicle_command.param4 = None
        #     self.send_vehicle_command.param5 = None
        #     self.send_vehicle_command.param6 = None
        #     self.send_vehicle_command.param7 = None
        #     self.publisher_offboard_mode.publish(offboard_msg)

        #     self.state = "ARM"
        #     self.get_logger().info("Arming.")
        #     self.publisher_vehicle_command.publish(self.send_vehicle_command)
        #     return
        # elif (self.state == "OFFBOARD" and self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.arming_state == VehicleStatus.ARMING_STATE_ARMED):
        #     self.get_logger().info("Already Armed.")
        #     self.state = "ARM"
        # if (self.state == "ARM" and self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.arming_state == VehicleStatus.ARMING_STATE_ARMED):
        #     # Takeoff
        #     self.send_vehicle_command = VehicleCommand()
        #     self.send_vehicle_command.command = 22
        #           # MAV_CMD_NAV_TAKEOFF
        #     self.send_vehicle_command.param1 = None
        #     self.send_vehicle_command.param2 = None
        #     self.send_vehicle_command.param3 = None
        #     self.send_vehicle_command.param4 = 0.0
        #     self.send_vehicle_command.param5 = self.vehicle_global_position[0]
        #     self.send_vehicle_command.param6 = self.vehicle_global_position[1]
        #     self.send_vehicle_command.param7 = 10.0
        #     self.publisher_offboard_mode.publish(offboard_msg)

        #     self.state = "TAKEOFF"
        # if (self.state == "TAKEOFF" and self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.arming_state == VehicleStatus.ARMING_STATE_ARMED):
        #     # Setpoint
        #     self.get_logger().info("여기까지?")
        # OFFBOARD 모드 전환을 위해 반드시 주기적으로 퍼블리시
        # ✅ OFFBOARD 모드 유지 (주기적 퍼블리시)
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        self.publisher_offboard_mode.publish(offboard_msg)

        # ✅ OFFBOARD 모드 지속 유지 (TrajectorySetpoint 발행)
        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.position[0] = self.vehicle_global_position[0]  # 현재 위치 유지
        trajectory_msg.position[1] = self.vehicle_global_position[1]
        trajectory_msg.position[2] = -10.0  # 목표 고도 10m
        trajectory_msg.velocity[0] = 0.0
        trajectory_msg.velocity[1] = 0.0
        trajectory_msg.velocity[2] = 0.0
        self.publisher_trajectory.publish(trajectory_msg)

        if self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.send_vehicle_command = VehicleCommand()
            self.send_vehicle_command.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
            self.send_vehicle_command.param1 = 1.0  # Mode switch: 1 = PX4_CUSTOM_MAIN_MODE_OFFBOARD
            self.send_vehicle_command.param2 = 6.0  # Submode: 6 = PX4_CUSTOM_SUB_MODE_AUTO_LOITER (유지 모드)
            self.publisher_vehicle_command.publish(self.send_vehicle_command)
            self.get_logger().info("Trying to switch to OFFBOARD mode...")
            

        if self.arming_state != VehicleStatus.ARMING_STATE_ARMED:
            self.send_vehicle_command = VehicleCommand()
            self.send_vehicle_command.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
            self.send_vehicle_command.param1 = 1.0  # 1.0 = ARM
            self.publisher_vehicle_command.publish(self.send_vehicle_command)
            self.get_logger().info("Arming vehicle...")
            

        if self.state == "ARM":
            self.send_vehicle_command = VehicleCommand()
            self.send_vehicle_command.command = VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF
            self.send_vehicle_command.param1 = 0.0
            self.send_vehicle_command.param4 = 0.0
            self.send_vehicle_command.param5 = self.vehicle_global_position[0]
            self.send_vehicle_command.param6 = self.vehicle_global_position[1]
            self.send_vehicle_command.param7 = 10.0  # 10m 이륙
            self.publisher_vehicle_command.publish(self.send_vehicle_command)
            self.get_logger().info("Takeoff initiated.")
            self.state = "TAKEOFF"




def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
