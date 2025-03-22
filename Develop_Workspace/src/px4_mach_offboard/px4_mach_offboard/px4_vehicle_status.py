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

__author__ = "Haechan Lee"
__contact__ = "eojin333c@gmail.com"

from re import M
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from px4_msgs.msg import VehicleStatus, VehicleLocalPosition, VehicleGlobalPosition, VehicleOdometry
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path



class PX4Status(Node):
    def __init__(self):
        super().__init__("VehicleStatus")
        self.declare_parameter("path_clearing_timeout", 5.0)

        # Configure subscritpions
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self.vehicle_local_position_callback,
            qos_profile,
        )
        self.global_position_sub = self.create_subscription(
            VehicleGlobalPosition,
            "/fmu/out/vehicle_global_position",
            self.vehicle_global_position_callback,
            qos_profile,
        )        
        self.vehicle_odometry_sub= self.create_subscription(
            VehicleOdometry,
            "/fmu/out/vehicle_odometry",
            self.vehicle_odometry_callback,
            qos_profile,
        )
        self.status_sub = self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status",
            self.vehicle_status_callback,
            qos_profile,
        )

        self.last_local_pos_update = 0.0
        self.vehicle_path_msg = Path()  # 경로 메시지 추가
        self.vehicle_local_position = np.array([0.0, 0.0, 0.0])
        self.vehicle_global_position = np.array([0.0, 0.0, 0.0])
        self.vehicle_status = np.array([0.0, 0.0])
        self.vehicle_odometry = np.array([0.0, 0.0, 0.0, 0])
        self.NAVIGATION_STATE_MAP = {
            0: "Manual mode",
            1: "Altitude control mode",
            2: "Position control mode",
            3: "Auto mission mode",
            4: "Auto loiter mode",
            5: "Auto return to launch mode",
            6: "Position slow mode",
            10: "Acro mode",
            11: "Free mode 2",
            12: "Descend mode (no position control)",
            13: "Termination mode",
            14: "Offboard mode",
            15: "Stabilized mode",
            17: "Auto takeoff",
            18: "Auto land",
            19: "Auto Follow",
            20: "Precision land with landing target",
            21: "Orbit in a circle",
            22: "Takeoff, transition, establish loiter",
            23: "External mode 1",
        }
        self.create_timer(0.1, self.show_vehicle_position)
        

    def vehicle_local_position_callback(self, msg):
        path_clearing_timeout = (
            self.get_parameter("path_clearing_timeout")
            .get_parameter_value()
            .double_value
        )
        if path_clearing_timeout >= 0 and (
            (Clock().now().nanoseconds / 1e9 - self.last_local_pos_update)
            > path_clearing_timeout
        ):
            self.vehicle_path_msg.poses.clear()
        self.last_local_pos_update = Clock().now().nanoseconds / 1e9

        # TODO: handle NED->ENU transformation
        self.vehicle_local_position[0] = msg.x
        self.vehicle_local_position[1] = -msg.y
        self.vehicle_local_position[2] = -msg.z

    def vehicle_status_callback(self, msg):
        path_clearing_timeout = (
            self.get_parameter("path_clearing_timeout")
            .get_parameter_value()
            .double_value
        )
        if path_clearing_timeout >= 0 and (
            (Clock().now().nanoseconds / 1e9 - self.last_local_pos_update)
            > path_clearing_timeout
        ):
            self.vehicle_path_msg.poses.clear()
        self.last_local_pos_update = Clock().now().nanoseconds / 1e9

        # TODO: handle NED->ENU transformation
        self.vehicle_status[0] = msg.timestamp
        self.vehicle_status[1] = msg.nav_state
        
    def vehicle_odometry_callback(self, msg):
        path_clearing_timeout = (
            self.get_parameter("path_clearing_timeout")
            .get_parameter_value()
            .double_value
        )
        if path_clearing_timeout >= 0 and (
            (Clock().now().nanoseconds / 1e9 - self.last_local_pos_update)
            > path_clearing_timeout
        ):
            self.vehicle_path_msg.poses.clear()
        self.last_local_pos_update = Clock().now().nanoseconds / 1e9

        # TODO: handle NED->ENU transformation
        self.vehicle_odometry[0] = msg.velocity[0]
        self.vehicle_odometry[1] = msg.velocity[1]
        self.vehicle_odometry[2] = msg.velocity[2]
        self.vehicle_odometry[3] = msg.velocity_frame

    def vehicle_global_position_callback(self, msg):
        path_clearing_timeout = (
            self.get_parameter("path_clearing_timeout")
            .get_parameter_value()
            .double_value
        )
        if path_clearing_timeout >= 0 and (
            (Clock().now().nanoseconds / 1e9 - self.last_local_pos_update)
            > path_clearing_timeout
        ):
            self.vehicle_path_msg.poses.clear()
        self.last_local_pos_update = Clock().now().nanoseconds / 1e9

        # TODO: handle NED->ENU transformation
        self.vehicle_global_position[0] = msg.lat
        self.vehicle_global_position[1] = msg.lon
        self.vehicle_global_position[2] = msg.alt
        
    def get_nav_state_description(self, nav_status):
        return self.NAVIGATION_STATE_MAP.get(nav_status, "UNKNOWN")
    
    def show_vehicle_position(self):
        
        msg = f"""
        Vehicle Status
        =========================================
        TIMESTAMP: {self.vehicle_status[0]}
        NAV STATE: {self.get_nav_state_description(self.vehicle_status[1])} , {self.vehicle_status[1]}
        =========================================

        LOCAL POSITION 
            X: {self.vehicle_local_position[0]}
            Y: {self.vehicle_local_position[1]}
            Z: {self.vehicle_local_position[2]}

        GLOBAL POSITION 
            LAT: {self.vehicle_global_position[0]}
            LON: {self.vehicle_global_position[1]}
            ALT: {self.vehicle_global_position[2]}
        
        =========================================
         VELOCITY (meter/sec) Reference frame : {self.vehicle_odometry[3]}(NED)
            North : {self.vehicle_odometry[0]}
            East : {self.vehicle_odometry[1]}
            Down : {self.vehicle_odometry[2]}
        =========================================
        """
        self.get_logger().info(msg)
        






def main(args=None):
    rclpy.init(args=args)

    px4_status = PX4Status()

    rclpy.spin(px4_status)

    px4_status.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
