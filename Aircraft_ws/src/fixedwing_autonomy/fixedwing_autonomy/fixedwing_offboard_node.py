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

        # 추가
        self.local_vz = float('nan')
        
        # 현재 지면 기준 높이
        self.current_agl_m = float('nan')
        self.agl_source = 'invalid'
        
        # OFFBOARD 진입 순간의 지면 기준 높이
        self.offboard_entry_agl_m = float('nan')
        
        # 현재 WP의 목표 지면 기준 높이
        self.hold_agl_m = float('nan')

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
        self.target_airspeed_mps = 15.0

        # 게이트 하단 2.5m + 세로 높이 5m의 절반
        # = 게이트 중심 5.0m AGL
        self.gate_target_agl_m = 12.0
        
        # OFFBOARD 진입 시 한 번만 계산할 지면 기준값
        self.ground_altitude_amsl = float('nan')
        self.ground_local_z = float('nan')
        
        # 로그 표시용
        self.hold_agl_m = float('nan')

        # OFFBOARD 진입 당시 고도를 기준으로
        # 게이트 구간에서 적용할 고도 보정량
        #
        # -3.0: 현재 고도보다 3m 하강
        # +3.0: 현재 고도보다 3m 상승
        self.gate_altitude_adjust_m = -3.0
        
        
        # 고정 local NED waypoint
        #
        # 현재 시뮬레이션에서 기체 전방이 +y 방향이므로
        # x는 0으로 유지하고 y를 증가시킨다.
        #
        # WP1→WP2 = 100m
        # WP2→WP3 = 50m
        # WP3→WP4 = 30m
        self.fixed_waypoint_plan = [
            {
                'name': 'WP1_APPROACH',
                'x': 0.0,
                'y': 400.0,
        
            },
            {
                'name': 'WP2_RECOGNITION',
                'x': 0.0,
                'y': 500.0,
        
            },
            {
                'name': 'WP3_GATE_EXIT',
                'x': 0.0,
                'y': 550.0,
                
            },
            {
                'name': 'WP4_MISSION_END',
                'x': 0.0,
                'y': 580.0,
        
            },
        ]

        self.waypoint_acceptance_radius_m = 8.0
        # 고정익이 waypoint를 옆으로 지나쳤을 때
        # 통과로 인정할 최대 수평거리
        self.waypoint_pass_corridor_m = 20.0
        self.waypoint_altitude_acceptance_m = 5.0

        
        # 10 Hz 제어 루프에서 2회 연속 도착 조건을 만족해야 인정
        self.waypoint_reached_confirm_required = 2
        self.waypoint_reached_confirm_count = 0
        self.debug_log_count = 0
        
        # 전체 WP1~WP4 미션 제한 시간
        self.offboard_test_timeout_sec = 120.0
        
        self.waypoints = []
        self.current_waypoint_index = 0

        # OFFBOARD 진입 순간의 위치
        # WP1 통과선 판정에 사용한다.
        self.route_start_x = float('nan')
        self.route_start_y = float('nan')
                
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
        self.local_vz = msg.vz

        # PX4가 실제 지면까지의 거리를 계산하고 있다면
        # dist_bottom을 AGL로 우선 사용한다.
        if (
            msg.dist_bottom_valid
            and math.isfinite(msg.dist_bottom)
            and msg.dist_bottom >= 0.0
        ):
            self.current_agl_m = float(
                msg.dist_bottom
            )
            self.agl_source = 'dist_bottom'
    
        # dist_bottom이 없다면 local NED z 사용
        # z는 Down 방향이 양수이므로 비행 중 일반적으로 음수다.
        else:
            self.current_agl_m = float(
                -msg.z
            )
            self.agl_source = 'local_z'

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
            f'height AGL    : '
            f'{self.current_agl_m:.2f} m '
            f'[{self.agl_source}]\n'
            f'vertical speed: '
            f'{self.local_vz:.2f} m/s '
            f'[Down positive]\n'
            f'global GPS    : '
            f'lat={self.latitude:.7f}, '
            f'lon={self.longitude:.7f}, '
            f'alt_AMSL={self.altitude:.2f} m'
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
            if (
                self.nav_state
                == VehicleStatus.NAVIGATION_STATE_OFFBOARD
            ):
                if (
                    not math.isfinite(self.current_agl_m)
                    or not math.isfinite(self.altitude)
                    or not math.isfinite(self.local_z)
                ):
                    self.get_logger().error(
                        'Cannot finalize route: '
                        'AGL, AMSL, or local z is invalid.'
                    )
                    return
                
                
                # 현재 위치에서 지면 AMSL 계산
                #
                # 예:
                # 현재 AMSL 45m
                # 현재 AGL 20m
                # 지면 AMSL = 25m
                self.ground_altitude_amsl = (
                    self.altitude
                    - self.current_agl_m
                )
                
                
                # 로컬 NED 좌표에서 지면 z 계산
                #
                # 예:
                # 현재 local_z = -20
                # 현재 AGL = 20
                # ground_local_z = 0
                self.ground_local_z = (
                    self.local_z
                    + self.current_agl_m
                )
                
                self.get_logger().info(
                    'ALTITUDE TARGET CALC | '
                    f'current_AMSL={self.altitude:.2f}m | '
                    f'current_AGL={self.current_agl_m:.2f}m | '
                    f'ground_AMSL={self.ground_altitude_amsl:.2f}m | '
                    f'target_AGL={self.gate_target_agl_m:.2f}m | '
                    f'target_AMSL='
                    f'{self.ground_altitude_amsl + self.gate_target_agl_m:.2f}m'
                )
                # 실제 Offboard 진입 순간의 현재 위치와 진행방향을
                # 기준으로 WP1~WP4를 최종 생성하고 이후에는 고정한다.
                if not self.initialize_waypoint_list(
                    log_result=True,
                ):
                    self.get_logger().error(
                        'Failed to finalize waypoint list.'
                    )
                    return
                
                # 최종 경로의 첫 번째 waypoint를 현재 목표로 로드
                self.current_waypoint_index = 0
                
                if not self.load_current_waypoint_target():
                    self.get_logger().error(
                        'Failed to load first waypoint target.'
                    )
                    return
                
                self.get_logger().info(
                    'First waypoint target loaded | '
                    f'target_AMSL={self.hold_altitude_amsl:.2f}m | '
                    f'target_AGL={self.hold_agl_m:.2f}m | '
                    f'target_z={self.target_z:.2f}m'
                )
                
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

            # 기본값: 아직 waypoint를 통과하지 않음
            passed_waypoint = False
            
            
            # WP1은 OFFBOARD 진입 위치를 이전 지점으로 사용한다.
            if self.current_waypoint_index == 0:
            
                previous_x = self.route_start_x
                previous_y = self.route_start_y
            
            
            # WP2 이후는 바로 이전 waypoint를 사용한다.
            else:
            
                previous_waypoint = self.waypoints[
                    self.current_waypoint_index - 1
                ]
            
                previous_x = previous_waypoint['x']
                previous_y = previous_waypoint['y']
            
            
            # 이전 지점 → 현재 WP 경로 벡터
            path_dx = (
                current_waypoint['x']
                - previous_x
            )
            
            path_dy = (
                current_waypoint['y']
                - previous_y
            )
            
            path_length = math.hypot(
                path_dx,
                path_dy,
            )
            
            
            if path_length > 1.0:
            
                # 현재 WP → 기체 위치 벡터
                aircraft_after_dx = (
                    self.local_x
                    - current_waypoint['x']
                )
            
                aircraft_after_dy = (
                    self.local_y
                    - current_waypoint['y']
                )
            
                # 양수이면 현재 WP 통과선을 넘어간 상태
                passed_plane_value = (
                    aircraft_after_dx * path_dx
                    + aircraft_after_dy * path_dy
                )
            
                passed_waypoint = (
                    passed_plane_value >= 0.0
                    and distance_m
                    <= self.waypoint_pass_corridor_m
                )

            

            altitude_error_m = abs(
                self.altitude
                - current_waypoint['altitude_amsl']
            )

            requires_altitude = (
                self.current_waypoint_index == 1
            )

            altitude_condition_met = (
                not requires_altitude
                or altitude_error_m
                <= self.waypoint_altitude_acceptance_m
            )
            
            if self.debug_log_count % 10 == 0:
                agl_error_m = (
                    self.hold_agl_m
                    - self.current_agl_m
                )
                
                self.get_logger().info(
                    f"Tracking {current_waypoint['name']} | "
                    f"distance={distance_m:.1f}m | "
                    f"current_AGL={self.current_agl_m:.1f}m | "
                    f"target_AGL={self.hold_agl_m:.1f}m | "
                    f"AGL_error={agl_error_m:+.1f}m | "
                    f"vz_down={self.local_vz:+.2f}m/s"
                )
                        
            elapsed_sec = (
                self.get_clock().now()
                - self.offboard_active_start_time
            ).nanoseconds / 1_000_000_000
        
            waypoint_condition_met = (
                distance_m
                <= self.waypoint_acceptance_radius_m
                or passed_waypoint
            )            
            
            if waypoint_condition_met:
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
                    f'All {len(self.waypoints)} waypoints completed. '
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
    
    def initialize_waypoint_list(
        self,
        *,
        log_result: bool = True,
    ) -> bool:
        """
        고정 local NED x/y 좌표를 사용하고,
        현재 비행 고도를 기준으로 각 WP의 목표 고도를 생성한다.
        """
    
        if (
            math.isnan(self.local_x)
            or math.isnan(self.local_y)
            or math.isnan(self.local_z)
            or math.isnan(self.altitude)
        ):
            if log_result:
                self.get_logger().warning(
                    'Cannot initialize fixed waypoint route: '
                    'position or altitude is invalid.'
                )
    
            return False
    
        # 실제 OFFBOARD 전환 직전에 호출된 현재 위치를
        # WP1 통과선 판정용 경로 시작점으로 저장한다.
        self.route_start_x = self.local_x
        self.route_start_y = self.local_y
    
        # 실제 OFFBOARD 진입 후 고정된 지면 기준이 있으면 사용
        if (
            math.isfinite(
                self.ground_altitude_amsl
            )
            and math.isfinite(
                self.ground_local_z
            )
        ):
            ground_altitude_amsl = (
                self.ground_altitude_amsl
            )
        
            ground_local_z = (
                self.ground_local_z
            )
        
        # OFFBOARD 진입 전에는 현재 위치로 임시 지면 기준 계산
        elif (
            math.isfinite(self.altitude)
            and math.isfinite(self.local_z)
            and math.isfinite(self.current_agl_m)
        ):
            ground_altitude_amsl = (
                self.altitude
                - self.current_agl_m
            )
        
            ground_local_z = (
                self.local_z
                + self.current_agl_m
            )
        
            if log_result:
                self.get_logger().info(
                    'Using temporary ground reference | '
                    f'ground_AMSL={ground_altitude_amsl:.2f}m | '
                    f'ground_local_z={ground_local_z:.2f}m'
                )
        
        else:
            if log_result:
                self.get_logger().warning(
                    'Cannot estimate ground reference: '
                    'AGL, AMSL, or local z is invalid.'
                )
        
            return False        
        if math.isfinite(
            self.offboard_entry_agl_m
        ):
            base_agl_m = (
                self.offboard_entry_agl_m
            )
        else:
            # OFFBOARD 진입 전 임시 경로 생성용
            base_agl_m = (
                self.current_agl_m
            )
    
        self.waypoints = []
    
        for plan in self.fixed_waypoint_plan:
    
            waypoint_x = float(plan['x'])
            waypoint_y = float(plan['y'])

            # 안전하게 음수 고도가 생성되지 않도록 제한
            target_agl_m = max(
                5.0,
                float(self.gate_target_agl_m),
            )
            
            waypoint_altitude_amsl = (
                ground_altitude_amsl
                + target_agl_m
            )

            waypoint_z = (
                ground_local_z
                - target_agl_m
            )

            
            waypoint = {
                'name': plan['name'],
                'x': waypoint_x,
                'y': waypoint_y,
                'z': waypoint_z,
                'altitude_amsl': (
                    waypoint_altitude_amsl
                ),
                'target_agl_m': ( 
                    target_agl_m
                ),
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
                'Fixed-coordinate WP1~WP4 route initialized.'
            )
    
            self.get_logger().info(
                f'Route start | '
                f'x={self.route_start_x:.1f}, '
                f'y={self.route_start_y:.1f}, '
                f'altitude={ground_altitude_amsl:.1f}m AMSL'
            )
    
            for waypoint in self.waypoints:
                self.get_logger().info(
                    f"{waypoint['name']} | "
                    f"x={waypoint['x']:.1f}, "
                    f"y={waypoint['y']:.1f}, "
                    f"z={waypoint['z']:.1f}, "
                    f"altitude="
                    f"{waypoint['altitude_amsl']:.1f}m AMSL"
                )
    
            self.get_logger().info(
                'Now tracking WP1_APPROACH.'
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
        self.hold_agl_m = waypoint['target_agl_m']
    
        if log_result:
            self.get_logger().info(
                'ALTITUDE CHECK | '
                f'entry_AMSL={self.altitude:.2f} | '
                f'ground_AMSL={self.ground_altitude_amsl:.2f} | '
                f'calculated_target_AMSL='
                f'{self.ground_altitude_amsl + self.gate_target_agl_m:.2f} | '
                f'loaded_target_AMSL={self.hold_altitude_amsl:.2f}'
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