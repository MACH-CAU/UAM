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
    TrajectorySetpoint,
    FixedWingLateralSetpoint,
    FixedWingLongitudinalSetpoint
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

        # WP1~WP4 직선 경로 시험 설정
        self.waypoint_forward_distances_m = [
            150.0,
            250.0,
            350.0,
            450.0,
        ]
        
        self.waypoint_acceptance_radius_m = 25.0
        
        # 10 Hz 제어 루프에서 3회 연속 도착 조건을 만족해야 인정
        self.waypoint_reached_confirm_required = 3
        self.waypoint_reached_confirm_count = 0
        
        # 전체 WP1~WP4 미션 제한 시간
        self.offboard_test_timeout_sec = 90.0
        
        self.waypoints = []
        self.current_waypoint_index = 0
        
        # publish_trajectory_setpoint()와의 호환을 위해 유지
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

        #self.keyboard_thread = threading.Thread(
        #    target=self.wait_for_start_key,
        #    daemon=True,
        #)
        #self.keyboard_thread.start()
      

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
            # Arm 상태이며 고도 10 m 이상이면 자동비행 준비를 시작한다.
            # 실제 Offboard 진입은 RC/QGC 모드 입력이 담당한다.
            if airborne_manual_ready:
                if not self.initialize_waypoint_list(log_result=True):
                    return
        
                self.offboard_warmup_count = 0
                self.mission_state = 'OFFBOARD_WARMUP'
        
                self.get_logger().info(
                    'State: OFFBOARD_WARMUP — '
                    'preparing external setpoints.'
                )

        elif self.mission_state == 'OFFBOARD_WARMUP':
            # 수동비행 중 기체가 계속 이동하므로,
            # 현재 위치와 진행방향 기준으로 WP를 갱신한다.
            if not self.initialize_waypoint_list(log_result=False):
                return    
            
            self.offboard_warmup_count += 1    
            
            if self.offboard_warmup_count >= 20:
                self.mission_state = 'WAIT_FOR_RC_OFFBOARD'    
                self.get_logger().info(
                    'State: WAIT_FOR_RC_OFFBOARD — '
                    'select OFFBOARD using QGC or the RC switch.'
                )
        
        elif self.mission_state == 'WAIT_FOR_RC_OFFBOARD':
            # Offboard 진입 전까지 목표점을 현재 진행방향 앞쪽으로
            # 계속 갱신하여, 오래 기다려도 WP가 뒤쪽에 남지 않게 한다.
            if not self.initialize_waypoint_list(log_result=False):
                return
        
            # 노드가 모드 변경 명령을 보내지 않는다.
            # QGC 또는 실제 RC 스위치가 PX4를 Offboard로 전환하면
            # VehicleStatus의 nav_state 변화로 진입을 확인한다.
            if (
                self.nav_state
                == VehicleStatus.NAVIGATION_STATE_OFFBOARD
            ):
                # 실제 Offboard 진입 순간의 현재 위치와 진행방향을
                # 기준으로 WP1~WP4를 최종 생성하고 이후에는 고정한다.
                if not self.initialize_waypoint_list(
                    log_result=True,
                ):
                    self.get_logger().error(
                        'Failed to finalize waypoint list.'
                    )
                    return
                self.publish_fixed_wing_setpoints()

                self.offboard_active_start_time = (
                    self.get_clock().now()
                )
            
                self.mission_state = 'OFFBOARD_ACTIVE'
            
                self.get_logger().info(
                    'State: OFFBOARD_ACTIVE — '
                    'Offboard activated externally.'
                )
                    
        elif self.mission_state == 'REQUEST_STABILIZED':
            # 아직 Offboard이면 마지막 WP 명령을 유지한다.
            if (
                self.nav_state
                == VehicleStatus.NAVIGATION_STATE_OFFBOARD
            ):
                self.publish_fixed_wing_setpoints()
        
            # Hold, Position, Stabilized 등으로 실제 전환된 뒤 종료
            else:
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
            # 조종자가 언제든 다른 모드로 전환하면
            # 자동비행을 즉시 종료한다.
            if (
                self.nav_state
                != VehicleStatus.NAVIGATION_STATE_OFFBOARD
            ):
                self.mission_state = 'COMPLETE'
        
                self.get_logger().warning(
                    'State: COMPLETE — '
                    'pilot exited Offboard externally.'
                )
                return
            
            if self.failsafe is True:
                self.get_logger().error(
                    'Failsafe detected during waypoint mission. '
                    'Waiting for manual or automatic PX4 takeover.'
                )
            
                self.mission_state = 'REQUEST_STABILIZED'
                return
                    
            self.publish_fixed_wing_setpoints()
        
            distance_m = (
                self.distance_to_current_waypoint()
            )
        
            current_waypoint = self.waypoints[
                self.current_waypoint_index
            ]
        
            self.get_logger().info(
                f"Distance to {current_waypoint['name']}: "
                f'{distance_m:.1f} m'
            )
        
            elapsed_sec = (
                self.get_clock().now()
                - self.offboard_active_start_time
            ).nanoseconds / 1_000_000_000
        
            # 도착 반경 안에 연속으로 들어오는지 확인
            if (
                distance_m
                <= self.waypoint_acceptance_radius_m
            ):
                self.waypoint_reached_confirm_count += 1
            else:
                self.waypoint_reached_confirm_count = 0
        
            # 3회 연속 도착 조건 만족
            if (
                self.waypoint_reached_confirm_count
                >= self.waypoint_reached_confirm_required
            ):
                reached_name = current_waypoint['name']
        
                self.get_logger().info(
                    f'{reached_name} reached.'
                )
        
                if self.advance_to_next_waypoint():
                    return
        
                # 다음 WP가 없으면 WP4까지 완료
                self.get_logger().info(
                    'WP1~WP4 mission completed. '
                    'Switch to POSITION, HOLD, or STABILIZED manually.'
                )
        
                self.mission_state = 'REQUEST_STABILIZED'
                return
        
            if elapsed_sec >= self.offboard_test_timeout_sec:
                self.get_logger().warning(
                    'Waypoint mission timeout. '
                    'Switch to POSITION or HOLD manually.'
                )
        
                # 노드가 직접 모드 명령을 보내지 않는다.
                self.mission_state = 'REQUEST_STABILIZED'
    
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
    
    def initialize_waypoint_list(
        self,
        *,
        log_result: bool = True,
    ) -> bool:
        """
        현재 위치와 진행방향을 기준으로
        WP1~WP4 직선 경로를 생성한다.
        """
        horizontal_speed = math.hypot(
            self.local_vx,
            self.local_vy,
        )
    
        if (
            math.isnan(self.local_x)
            or math.isnan(self.local_y)
            or math.isnan(self.local_z)
            or math.isnan(self.local_vx)
            or math.isnan(self.local_vy)
            or math.isnan(self.altitude)
            or horizontal_speed < 3.0
        ):
            self.get_logger().warning(
                'Cannot create waypoint list: '
                'local position or velocity is invalid.'
            )
            return False
    
        direction_north = self.local_vx / horizontal_speed
        direction_east = self.local_vy / horizontal_speed
    
        start_x = self.local_x
        start_y = self.local_y
        start_z = self.local_z
    
        self.waypoints = []
    
        for index, distance_m in enumerate(
            self.waypoint_forward_distances_m
        ):
            waypoint = {
                'name': f'WP{index + 1}',
                'x': start_x + distance_m * direction_north,
                'y': start_y + distance_m * direction_east,
                'z': start_z,
                'altitude_amsl': self.altitude,
            }
    
            self.waypoints.append(waypoint)
    
        self.current_waypoint_index = 0
        self.waypoint_reached_confirm_count = 0
        self.waypoint_initialized = True
    
        self.load_current_waypoint_target(
            log_result=False,
        )
    
        if log_result:
            self.get_logger().info(
                'WP1~WP4 waypoint list initialized.'
            )
    
            for waypoint in self.waypoints:
                self.get_logger().info(
                    f"{waypoint['name']}: "
                    f"x={waypoint['x']:.1f}, "
                    f"y={waypoint['y']:.1f}, "
                    f"z={waypoint['z']:.1f}"
                )
    
            self.get_logger().info(
                'Now tracking WP1.'
            )
    
        return True
    
    
    def load_current_waypoint_target(
        self,
        *,
        log_result: bool = True,
    ) -> bool:
        """
        현재 waypoint의 좌표를 target_x/y/z에 적용한다.
        """
        if (
            not self.waypoints
            or self.current_waypoint_index
            >= len(self.waypoints)
        ):
            return False
    
        waypoint = self.waypoints[
            self.current_waypoint_index
        ]
    
        self.target_x = waypoint['x']
        self.target_y = waypoint['y']
        self.target_z = waypoint['z']
        self.hold_altitude_amsl = waypoint['altitude_amsl']
    
        if log_result:
            self.get_logger().info(
                f"Now tracking {waypoint['name']}: "
                f"x={self.target_x:.1f}, "
                f"y={self.target_y:.1f}, "
                f"z={self.target_z:.1f}"
                f"alt={self.hold_altitude_amsl:.1f} m AMSL"
            )
    
        return True
    
    def advance_to_next_waypoint(self) -> bool:
        """
        다음 waypoint가 있으면 전환한다.
    
        반환값:
        True  = 다음 WP로 전환됨
        False = 마지막 WP까지 완료됨
        """
        next_index = self.current_waypoint_index + 1
    
        if next_index >= len(self.waypoints):
            return False
    
        self.current_waypoint_index = next_index
        self.waypoint_reached_confirm_count = 0
    
        self.load_current_waypoint_target(
            log_result=True,
        )
    
        return True
    
    def publish_fixed_wing_setpoints(self) -> None:
        """
        현재 위치에서 현재 WP를 향하는 course와
        유지 고도·목표 등가대기속도를 PX4에 전달한다.
        """
        if (
            not self.waypoint_initialized
            or math.isnan(self.local_x)
            or math.isnan(self.local_y)
            or math.isnan(self.local_vx)
            or math.isnan(self.local_vy)
            or math.isnan(self.hold_altitude_amsl)
        ):
            return
    
        dx = self.target_x - self.local_x
        dy = self.target_y - self.local_y
        distance_m = math.hypot(dx, dy)
    
        # WP와 거의 동일한 위치에서는 atan2(0, 0)을 피하고
        # 현재 진행방향을 유지한다.
        if distance_m >= 1.0:
            target_course_rad = math.atan2(dy, dx)
        else:
            horizontal_speed = math.hypot(
                self.local_vx,
                self.local_vy,
            )
    
            if horizontal_speed < 1.0:
                return
    
            target_course_rad = math.atan2(
                self.local_vy,
                self.local_vx,
            )
    
        timestamp_us = int(
            self.get_clock().now().nanoseconds / 1000
        )
    
        lateral_msg = FixedWingLateralSetpoint()
        lateral_msg.timestamp = timestamp_us
    
        # Local NED:
        # x = North, y = East이므로 atan2(East, North)가 course다.
        lateral_msg.course = float(target_course_rad)
    
        # course 제어만 사용
        lateral_msg.airspeed_direction = float('nan')
        lateral_msg.lateral_acceleration = float('nan')
    
        longitudinal_msg = FixedWingLongitudinalSetpoint()
        longitudinal_msg.timestamp = timestamp_us
    
        # 반드시 AMSL 고도
        longitudinal_msg.altitude = float(
            self.hold_altitude_amsl
        )
    
        # altitude 직접 제어를 사용하므로 height_rate는 NaN
        longitudinal_msg.height_rate = float('nan')
    
        longitudinal_msg.equivalent_airspeed = float(
            self.target_airspeed_mps
        )
    
        # PX4의 TECS가 pitch와 throttle을 계산하게 한다.
        longitudinal_msg.pitch_direct = float('nan')
        longitudinal_msg.throttle_direct = float('nan')
    
        self.fw_lateral_setpoint_pub.publish(
            lateral_msg
        )
    
        self.fw_longitudinal_setpoint_pub.publish(
            longitudinal_msg
        )
    
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
        """
        현재 waypoint까지의 수평 거리를 계산한다.
        """
        if (
            not self.waypoint_initialized
            or not self.waypoints
        ):
            return float('inf')
    
        dx = self.target_x - self.local_x
        dy = self.target_y - self.local_y
    
        return math.hypot(dx, dy)
    
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