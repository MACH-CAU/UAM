#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from px4_msgs.msg import (
    VehicleStatus,
    VehicleLocalPosition,
    VehicleGlobalPosition,
    VehicleCommand,
    VehicleCommandAck,
    OffboardControlMode,
    FixedWingLateralSetpoint,
    FixedWingLongitudinalSetpoint,
    TrajectorySetpoint,
)

from std_msgs.msg import Bool

import threading


class FixedWingOffboardNode(Node):

    def __init__(self) -> None:
        super().__init__('fixedwing_offboard_node')

        # PX4 uXRCE-DDS 토픽용 QoS
        px4_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # 현재 비행 상태 저장
        self.nav_state = None
        self.arming_state = None
        self.failsafe = None

        # 현재 로컬 NED 위치 저장
        self.local_x = float('nan')
        self.local_y = float('nan')
        self.local_z = float('nan')
        self.local_vx = float('nan')
        self.local_vy = float('nan')

        # 현재 GPS 위치 저장
        self.latitude = float('nan')
        self.longitude = float('nan')
        self.altitude = float('nan')
        
        self.command_ack_received = False
        self.test_start_time = self.get_clock().now()
        self.offboard_counter = 0
        self.offboard_command_sent = False
        
        self.mission_state = 'WAIT_FOR_AIRBORNE_MANUAL'
        self.start_mission_requested = False
        self.offboard_command_sent = False
        self.offboard_warmup_count = 0

        self.hold_course_rad = float('nan')
        self.hold_altitude_amsl = float('nan')
        self.target_airspeed_mps = 18.0

        # WP1~WP4 시험 설정: 미션 시작 위치에서 진행방향 기준 상대거리
        self.waypoint_forward_distances_m = [40.0, 80.0, 120.0, 160.0]
        self.waypoint_acceptance_radius_m = 15.0
        self.waypoint_reached_confirm_count = 3
        self.offboard_test_timeout_sec = 60.0

        self.waypoints = []
        self.current_waypoint_index = 0
        self.waypoint_reached_count = 0

        self.target_x = float('nan')
        self.target_y = float('nan')
        self.target_z = float('nan')

        self.waypoint_initialized = False
        self.offboard_active_start_time = None

        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v4',
            self.vehicle_status_callback,
            px4_qos,
        )

        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.local_position_callback,
            px4_qos,
        )

        self.global_position_sub = self.create_subscription(
            VehicleGlobalPosition,
            '/fmu/out/vehicle_global_position',
            self.global_position_callback,
            px4_qos,
        )

        self.command_ack_sub = self.create_subscription(
            VehicleCommandAck,
            "/fmu/out/vehicle_command_ack",
            self.command_ack_callback,
            px4_qos,
        )        


        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            "/fmu/in/vehicle_command",
            10,
        )
        
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10,
        )
        
        self.command_ack_sub = self.create_subscription(
            VehicleCommandAck,
            '/fmu/out/vehicle_command_ack_v1',
            self.command_ack_callback,
            px4_qos,
        )

        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10,
        )

        self.fw_lateral_setpoint_pub = self.create_publisher(
            FixedWingLateralSetpoint,
            '/fmu/in/fixed_wing_lateral_setpoint',
            10,
        )
        
        self.fw_longitudinal_setpoint_pub = self.create_publisher(
            FixedWingLongitudinalSetpoint,
            '/fmu/in/fixed_wing_longitudinal_setpoint',
            10,
        )
    
        self.start_mission_sub = self.create_subscription(
            Bool,
            '/start_mission',
            self.start_mission_callback,
            10,
        )   

        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10,
        )

        # 1초마다 현재 상태 출력
        self.print_timer = self.create_timer(
            1.0,
            self.print_vehicle_state,
        )

        self.get_logger().info(
            'Fixed-wing offboard node started. Waiting for PX4 data...'
        )
        
        self.control_timer = self.create_timer(
            0.1,
            self.control_loop,
        )

        self.keyboard_thread = threading.Thread(
            target=self.wait_for_start_key,
            daemon=True,
        )
        self.keyboard_thread.start()
      

    def vehicle_status_callback(self, msg: VehicleStatus) -> None:
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state
        self.failsafe = msg.failsafe

    def local_position_callback(
        self,
        msg: VehicleLocalPosition,
    ) -> None:
        self.local_x = msg.x
        self.local_y = msg.y
        self.local_z = msg.z
        self.local_vx = msg.vx
        self.local_vy = msg.vy

    def global_position_callback(
        self,
        msg: VehicleGlobalPosition,
    ) -> None:
        self.latitude = msg.lat
        self.longitude = msg.lon
        self.altitude = msg.alt

    def print_vehicle_state(self) -> None:
        self.get_logger().info(
            '\n'
            f'nav_state     : {self.nav_state}\n'
            f'arming_state  : {self.arming_state}\n'
            f'failsafe      : {self.failsafe}\n'
            f'local NED     : '
            f'x={self.local_x:.2f}, '
            f'y={self.local_y:.2f}, '
            f'z={self.local_z:.2f}\n'
            f'global GPS    : '
            f'lat={self.latitude:.7f}, '
            f'lon={self.longitude:.7f}, '
            f'alt={self.altitude:.2f}'
        )
    
    def command_ack_callback(
        self,
        msg: VehicleCommandAck,
    ) -> None:
        result_names = {
            0: 'ACCEPTED',
            1: 'TEMPORARILY_REJECTED',
            2: 'DENIED',
            3: 'UNSUPPORTED',
            4: 'FAILED',
            5: 'IN_PROGRESS',
            6: 'CANCELLED',
        }
    
        result_text = result_names.get(
            msg.result,
            f'UNKNOWN({msg.result})',
        )
    
        self.get_logger().info(
            f'COMMAND ACK | '
            f'command={msg.command}, '
            f'result={result_text}'
        )
    
    def wait_for_start_key(self) -> None:
        input('Press Enter to start mission...\n')
        self.start_mission_requested = True
        self.get_logger().info('Mission start requested by keyboard.')

    def publish_vehicle_command(
        self,
        command: int,
        *,
        param1: float = 0.0,
        param2: float = 0.0,
    ) -> None:
        msg = VehicleCommand()
    
        msg.timestamp = int(
            self.get_clock().now().nanoseconds / 1000
        )
    
        msg.param1 = param1
        msg.param2 = param2
    
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
    
        self.vehicle_command_pub.publish(msg)
    
        self.get_logger().info(
            f'VehicleCommand sent: command={command}, '
            f'param1={param1}'
        )
    
    def request_offboard_mode(self) -> None:
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0,
        )
    
        self.get_logger().info(
            'OFFBOARD mode command sent.'
        )
    
    def request_offboard_mode(self) -> None:
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0,
        )
        self.get_logger().info('OFFBOARD mode command sent.')
    
    
    def request_stabilized_mode(self) -> None:
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=7.0,
        )
        self.get_logger().info('STABILIZED mode command sent.')
        def arm(self) -> None:
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                param1=1.0,
            )
            self.get_logger().info('ARM command sent.')
          
      
    def disarm(self) -> None:
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=0.0,
        )
        self.get_logger().info('DISARM command sent.')
    
    def command_test_callback(self) -> None:
        elapsed_seconds = (
            self.get_clock().now() - self.test_start_time
        ).nanoseconds / 1_000_000_000
    
        # 노드 시작 3초 후 ARM
        if elapsed_seconds >= 3.0 and not self.arm_command_sent:
            self.arm()
            self.arm_command_sent = True
            return
    
        # 노드 시작 8초 후 DISARM
        if elapsed_seconds >= 8.0 and not self.disarm_command_sent:
            self.disarm()
            self.disarm_command_sent = True
    
            self.get_logger().info(
                'ARM/DISARM command test completed.'
            )

    def publish_offboard_control_mode(self) -> None:
        msg = OffboardControlMode()
    
        msg.timestamp = int(
            self.get_clock().now().nanoseconds / 1000
        )
    
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False
    
        self.offboard_control_mode_pub.publish(msg)
    
    def control_loop(self) -> None:
        # Offboard heartbeat는 계속 10 Hz로 보낸다.
        self.publish_offboard_control_mode()
    
        airborne_manual_ready = (
            self.arming_state
            == VehicleStatus.ARMING_STATE_ARMED
            and self.failsafe is False
            and not math.isnan(self.local_z)
            and self.local_z < -10.0
        )
    
        if self.mission_state == 'WAIT_FOR_AIRBORNE_MANUAL':
            if self.start_mission_requested and airborne_manual_ready:
                if not self.initialize_waypoint_list():
                    self.start_mission_requested = False
                    return
    
                self.offboard_warmup_count = 0
                self.mission_state = 'OFFBOARD_WARMUP'
                self.get_logger().info('State: OFFBOARD_WARMUP')
    
        elif self.mission_state == 'OFFBOARD_WARMUP':
            self.publish_trajectory_setpoint()
            self.offboard_warmup_count += 1
    
            if self.offboard_warmup_count >= 20:
                self.request_offboard_mode()
                self.mission_state = 'REQUEST_OFFBOARD'
                self.get_logger().info('State: REQUEST_OFFBOARD')
        
        elif self.mission_state == 'REQUEST_STABILIZED':
            self.publish_trajectory_setpoint()
        
            if self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.mission_state = 'COMPLETE'
                self.get_logger().info(
                    'State: COMPLETE — pilot control restored.'
                )
    
        elif self.mission_state == 'REQUEST_OFFBOARD':
            self.publish_trajectory_setpoint()
    
            if (
                self.nav_state
                == VehicleStatus.NAVIGATION_STATE_OFFBOARD
            ):
                self.offboard_active_start_time = self.get_clock().now()
                self.mission_state = 'OFFBOARD_ACTIVE'
                self.get_logger().info('State: OFFBOARD_ACTIVE')
    
        elif self.mission_state == 'OFFBOARD_ACTIVE':
            self.publish_trajectory_setpoint()

            distance_m = self.distance_to_current_waypoint()
            wp_number = self.current_waypoint_index + 1

            self.get_logger().info(
                f'Distance to WP{wp_number}: {distance_m:.1f} m'
            )

            elapsed_sec = (
                self.get_clock().now()
                - self.offboard_active_start_time
            ).nanoseconds / 1_000_000_000

            if distance_m <= self.waypoint_acceptance_radius_m:
                self.waypoint_reached_count += 1
            else:
                self.waypoint_reached_count = 0

            if (
                self.waypoint_reached_count
                >= self.waypoint_reached_confirm_count
            ):
                self.advance_to_next_waypoint()

            elif elapsed_sec >= self.offboard_test_timeout_sec:
                self.get_logger().warning(
                    'Waypoint mission timeout. Switch to STABILIZED manually.'
                )
                self.mission_state = 'REQUEST_STABILIZED'
    
        elif self.mission_state == 'REQUEST_STABILIZED':
            # 전환 완료 전까지 heartbeat는 계속 보내지만
            # 더 이상 새로운 경로는 진행하지 않는다.
            if (
                self.nav_state
                == VehicleStatus.NAVIGATION_STATE_STAB
            ):
                self.mission_state = 'COMPLETE'
                self.get_logger().info(
                    'State: COMPLETE — pilot control restored.'
                )
    
        elif self.mission_state == 'COMPLETE':
            pass
    
    def start_mission_callback(self, msg: Bool) -> None:
        if msg.data:
    
            self.publish_fixed_wing_setpoints()
        
            self.offboard_warmup_count += 1
        
            if self.offboard_warmup_count >= 20:
                self.request_offboard_mode()
                self.mission_state = 'REQUEST_OFFBOARD'
                self.get_logger().info('State: REQUEST_OFFBOARD')
    
        elif self.mission_state == 'REQUEST_OFFBOARD':
            self.publish_fixed_wing_setpoints()
        
            if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.mission_state = 'OFFBOARD_ACTIVE'
                self.get_logger().info('State: OFFBOARD_ACTIVE')
            
        elif self.mission_state == 'OFFBOARD_ACTIVE':
            self.publish_fixed_wing_setpoints()
    
    def start_mission_callback(self, msg: Bool) -> None:
        if msg.data:
            self.start_mission_requested = True
            self.get_logger().info('Mission start requested.')
    
    def get_current_course(self) -> float:
        speed_horizontal = math.hypot(self.local_vx, self.local_vy)
    
        if speed_horizontal < 1.0:
            return float('nan')
    
        return math.atan2(self.local_vy, self.local_vx)
    
    def publish_fixed_wing_setpoints(self) -> None:
        timestamp_us = int(
            self.get_clock().now().nanoseconds / 1000
        )
    
        lateral_msg = FixedWingLateralSetpoint()
        lateral_msg.timestamp = timestamp_us
        lateral_msg.course = self.hold_course_rad
        lateral_msg.airspeed_direction = float('nan')
        lateral_msg.lateral_acceleration = float('nan')
    
        longitudinal_msg = FixedWingLongitudinalSetpoint()
        longitudinal_msg.timestamp = timestamp_us
        longitudinal_msg.altitude = self.hold_altitude_amsl
        longitudinal_msg.height_rate = float('nan')
        longitudinal_msg.equivalent_airspeed = self.target_airspeed_mps
        longitudinal_msg.pitch_direct = float('nan')
        longitudinal_msg.throttle_direct = float('nan')
    
        self.fw_lateral_setpoint_pub.publish(lateral_msg)
        self.fw_longitudinal_setpoint_pub.publish(longitudinal_msg)
    def initialize_waypoint_list(self) -> bool:
        """현재 위치와 진행방향 기준으로 WP1~WP4 로컬 NED 좌표를 생성한다."""
        horizontal_speed = math.hypot(self.local_vx, self.local_vy)

        if (
            math.isnan(self.local_x)
            or math.isnan(self.local_y)
            or math.isnan(self.local_z)
            or horizontal_speed < 3.0
        ):
            self.get_logger().warning(
                'Cannot create waypoint list: local position or velocity is invalid.'
            )
            return False

        direction_north = self.local_vx / horizontal_speed
        direction_east = self.local_vy / horizontal_speed

        self.waypoints = []
        for distance_m in self.waypoint_forward_distances_m:
            self.waypoints.append({
                'x': self.local_x + distance_m * direction_north,
                'y': self.local_y + distance_m * direction_east,
                'z': self.local_z,
            })

        self.current_waypoint_index = 0
        self.waypoint_reached_count = 0
        self.waypoint_initialized = True
        self.load_current_waypoint_target()

        for index, waypoint in enumerate(self.waypoints, start=1):
            self.get_logger().info(
                f'WP{index} initialized: '
                f'x={waypoint["x"]:.1f}, '
                f'y={waypoint["y"]:.1f}, '
                f'z={waypoint["z"]:.1f}'
            )

        return True

    def load_current_waypoint_target(self) -> None:
        """현재 waypoint index의 좌표를 TrajectorySetpoint 목표값에 반영한다."""
        waypoint = self.waypoints[self.current_waypoint_index]
        self.target_x = waypoint['x']
        self.target_y = waypoint['y']
        self.target_z = waypoint['z']

        self.get_logger().info(
            f'Now tracking WP{self.current_waypoint_index + 1}: '
            f'x={self.target_x:.1f}, '
            f'y={self.target_y:.1f}, '
            f'z={self.target_z:.1f}'
        )

    def advance_to_next_waypoint(self) -> None:
        """현재 WP 도착 처리 후 다음 WP로 전환하거나 수동 인계를 기다린다."""
        reached_wp_number = self.current_waypoint_index + 1
        self.get_logger().info(f'WP{reached_wp_number} reached.')
        self.waypoint_reached_count = 0

        if self.current_waypoint_index < len(self.waypoints) - 1:
            self.current_waypoint_index += 1
            self.load_current_waypoint_target()
            return

        self.get_logger().info(
            'WP4 reached. Switch the RC mode switch to STABILIZED manually.'
        )
        self.mission_state = 'REQUEST_STABILIZED'

    def publish_trajectory_setpoint(self) -> None:
        if not self.waypoint_initialized:
            return
    
        msg = TrajectorySetpoint()
    
        msg.timestamp = int(
            self.get_clock().now().nanoseconds / 1000
        )
    
        msg.position = [
            float(self.target_x),
            float(self.target_y),
            float(self.target_z),
        ]
    
        msg.velocity = [
            float('nan'),
            float('nan'),
            float('nan'),
        ]
    
        msg.acceleration = [
            float('nan'),
            float('nan'),
            float('nan'),
        ]
    
        msg.jerk = [
            float('nan'),
            float('nan'),
            float('nan'),
        ]
    
        msg.yaw = float('nan')
        msg.yawspeed = float('nan')
    
        self.trajectory_setpoint_pub.publish(msg)
    
    def distance_to_current_waypoint(self) -> float:
        if not self.waypoint_initialized:
            return float('inf')
    
        dx = self.target_x - self.local_x
        dy = self.target_y - self.local_y
        dz = self.target_z - self.local_z
    
        return math.sqrt(dx * dx + dy * dy + dz * dz)
    
def main(args=None) -> None:
    rclpy.init(args=args)

    node = FixedWingOffboardNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()