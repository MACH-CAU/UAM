#!/usr/bin/env python3

"""
Fixed-wing Offboard waypoint node with dynamic vision-gate route insertion.

Gate solution topic
-------------------
Topic: /vision/gate_solution_ned
Type : std_msgs/msg/Float32MultiArray
Data : [valid, gate_x_north_m, gate_y_east_m, gate_agl_m,
        gate_course_rad, confidence]

- valid: 1.0 for a usable observation, 0.0 otherwise
- gate_x/y: PX4 local NED horizontal coordinates
- gate_agl_m: desired gate-center height above ground
- gate_course_rad: desired course through the gate, atan2(East, North)
  Use NaN to let this node derive the course from WP2 to WP3.
- confidence: localization confidence in [0, 1]

The node requires several mutually consistent gate solutions before replacing
GATE_SEARCH_FORWARD with GATE_APPROACH -> GATE_CENTER -> GATE_EXIT.
"""

import math
from collections import deque
from typing import Deque, Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from px4_msgs.msg import (
    AirspeedValidated,
    FixedWingLateralSetpoint,
    FixedWingLongitudinalSetpoint,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleCommandAck,
    VehicleGlobalPosition,
    VehicleLocalPosition,
    VehicleStatus,
)
from std_msgs.msg import Bool, Float32MultiArray


class FixedWingOffboardNode(Node):
    def __init__(self) -> None:
        super().__init__('fixedwing_offboard_node')

        px4_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ------------------------- Parameters -------------------------
        # Fallback speed is used only when a valid manual-flight airspeed
        # cannot be captured at the moment Offboard becomes active.
        self.declare_parameter('target_airspeed_mps', 15.0)
        self.declare_parameter('capture_manual_airspeed', True)
        self.declare_parameter('minimum_mission_airspeed_mps', 10.0)
        self.declare_parameter('maximum_mission_airspeed_mps', 18.0)
        self.declare_parameter('airspeed_capture_window_sec', 1.0)
        self.declare_parameter('airspeed_sample_max_age_sec', 1.0)
        self.declare_parameter('default_gate_agl_m', 12.0)
        self.declare_parameter('post_gate_agl_m', 20.0)
        self.declare_parameter('altitude_rate_kp', 0.35)
        self.declare_parameter('max_climb_rate_mps', 1.5)
        self.declare_parameter('max_sink_rate_mps', 2.0)
        self.declare_parameter('altitude_deadband_m', 0.4)
        self.declare_parameter('gate_confidence_min', 0.55)
        self.declare_parameter('gate_solution_confirm_required', 3)
        self.declare_parameter('gate_consistency_xy_m', 4.0)
        self.declare_parameter('gate_consistency_agl_m', 2.0)
        self.declare_parameter('gate_consistency_course_deg', 20.0)
        self.declare_parameter('gate_approach_distance_m', 15.0)
        self.declare_parameter('gate_exit_distance_m', 15.0)
        self.declare_parameter('gate_center_acceptance_radius_m', 1.5)
        self.declare_parameter('gate_center_pass_corridor_m', 7.0)
        self.declare_parameter('gate_center_altitude_tolerance_m', 1.5)
        self.declare_parameter('gate_solution_max_distance_m', 400.0)

        self.target_airspeed_mps = float(
            self.get_parameter('target_airspeed_mps').value
        )
        self.capture_manual_airspeed = bool(
            self.get_parameter('capture_manual_airspeed').value
        )
        self.minimum_mission_airspeed_mps = float(
            self.get_parameter('minimum_mission_airspeed_mps').value
        )
        self.maximum_mission_airspeed_mps = float(
            self.get_parameter('maximum_mission_airspeed_mps').value
        )
        self.airspeed_capture_window_sec = float(
            self.get_parameter('airspeed_capture_window_sec').value
        )
        self.airspeed_sample_max_age_sec = float(
            self.get_parameter('airspeed_sample_max_age_sec').value
        )

        if (
            self.minimum_mission_airspeed_mps
            > self.maximum_mission_airspeed_mps
        ):
            raise ValueError(
                'minimum_mission_airspeed_mps must not exceed '
                'maximum_mission_airspeed_mps.'
            )

        # The captured value is frozen once, at the Manual -> Offboard
        # transition. It is not continuously changed during the mission.
        self.mission_airspeed_mps = self.target_airspeed_mps
        self.default_gate_agl_m = float(
            self.get_parameter('default_gate_agl_m').value
        )
        self.post_gate_agl_m = float(
            self.get_parameter('post_gate_agl_m').value
        )
        self.altitude_rate_kp = float(
            self.get_parameter('altitude_rate_kp').value
        )
        self.max_climb_rate_mps = float(
            self.get_parameter('max_climb_rate_mps').value
        )
        self.max_sink_rate_mps = float(
            self.get_parameter('max_sink_rate_mps').value
        )
        self.altitude_deadband_m = float(
            self.get_parameter('altitude_deadband_m').value
        )
        self.gate_confidence_min = float(
            self.get_parameter('gate_confidence_min').value
        )
        self.gate_solution_confirm_required = int(
            self.get_parameter('gate_solution_confirm_required').value
        )
        self.gate_consistency_xy_m = float(
            self.get_parameter('gate_consistency_xy_m').value
        )
        self.gate_consistency_agl_m = float(
            self.get_parameter('gate_consistency_agl_m').value
        )
        self.gate_consistency_course_rad = math.radians(
            float(self.get_parameter('gate_consistency_course_deg').value)
        )
        self.gate_approach_distance_m = float(
            self.get_parameter('gate_approach_distance_m').value
        )
        self.gate_exit_distance_m = float(
            self.get_parameter('gate_exit_distance_m').value
        )
        self.gate_center_acceptance_radius_m = float(
            self.get_parameter('gate_center_acceptance_radius_m').value
        )
        self.gate_center_pass_corridor_m = float(
            self.get_parameter('gate_center_pass_corridor_m').value
        )
        self.gate_center_altitude_tolerance_m = float(
            self.get_parameter('gate_center_altitude_tolerance_m').value
        )
        self.gate_solution_max_distance_m = float(
            self.get_parameter('gate_solution_max_distance_m').value
        )

        # ------------------------- PX4 state -------------------------
        self.nav_state = None
        self.arming_state = None
        self.failsafe = None

        self.local_x = float('nan')
        self.local_y = float('nan')
        self.local_z = float('nan')
        self.local_vx = float('nan')
        self.local_vy = float('nan')
        self.local_vz = float('nan')

        self.current_airspeed_mps = float('nan')
        self.last_airspeed_rx_time_sec = float('nan')
        self.airspeed_samples: Deque[Tuple[float, float]] = deque(maxlen=100)

        self.current_agl_m = float('nan')
        self.agl_source = 'invalid'
        self.offboard_entry_agl_m = float('nan')

        self.latitude = float('nan')
        self.longitude = float('nan')
        self.altitude = float('nan')

        self.ground_altitude_amsl = float('nan')
        self.ground_local_z = float('nan')

        # ------------------------- Mission state -------------------------
        self.mission_state = 'WAIT_FOR_AIRBORNE_MANUAL'
        self.start_mission_requested = False
        self.offboard_warmup_count = 0
        self.offboard_active_start_time = None

        self.waypoint_acceptance_radius_m = 8.0
        self.waypoint_pass_corridor_m = 20.0
        self.waypoint_altitude_acceptance_m = 5.0
        self.waypoint_reached_confirm_required = 2
        self.waypoint_reached_confirm_count = 0
        self.offboard_test_timeout_sec = 180.0
        self.debug_log_count = 0

        # Base local-NED route. GATE_SEARCH_FORWARD is replaced after a
        # validated gate solution is committed.
        # Gate altitude is commanded from WP1 onward so the aircraft is
        # already vertically aligned before the short WP2-to-gate segment.
        self.fixed_waypoint_plan = [
            {
                'name': 'WP1_APPROACH',
                'x': 0.0,
                'y': 200.0,
                'target_agl_m': self.default_gate_agl_m,
            },
            {
                'name': 'WP2_RECOGNITION',
                'x': 0.0,
                'y': 300.0,
                'target_agl_m': self.default_gate_agl_m,
            },
            {
                'name': 'GATE_SEARCH_FORWARD',
                'x': 0.0,
                'y': 325.0,
                'target_agl_m': self.default_gate_agl_m,
            },
            {
                'name': 'WP3_GATE_EXIT',
                'x': 0.0,
                'y': 350.0,
                'target_agl_m': self.default_gate_agl_m,
            },
            {
                'name': 'WP4_MISSION_END',
                'x': 0.0,
                'y': 380.0,
                'target_agl_m': self.post_gate_agl_m,
            },
        ]

        self.waypoints: List[Dict[str, float]] = []
        self.current_waypoint_index = 0
        self.route_start_x = float('nan')
        self.route_start_y = float('nan')

        self.target_x = float('nan')
        self.target_y = float('nan')
        self.target_z = float('nan')
        self.hold_altitude_amsl = float('nan')
        self.hold_agl_m = float('nan')
        self.waypoint_initialized = False
        self.commanded_height_rate_mps = 0.0

        # ------------------------- Dynamic gate state -------------------------
        self.gate_solution_buffer: List[Dict[str, float]] = []
        self.confirmed_gate_solution: Optional[Dict[str, float]] = None
        self.gate_route_committed = False
        self.gate_route_state = 'WAITING'

        # ------------------------- ROS I/O -------------------------
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
        self.airspeed_sub = self.create_subscription(
            AirspeedValidated,
            '/fmu/out/airspeed_validated_v1',
            self.airspeed_callback,
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
            '/fmu/out/vehicle_command_ack_v1',
            self.command_ack_callback,
            px4_qos,
        )

        self.start_mission_sub = self.create_subscription(
            Bool,
            '/start_mission',
            self.start_mission_callback,
            10,
        )
        self.gate_solution_sub = self.create_subscription(
            Float32MultiArray,
            '/vision/gate_solution_ned',
            self.gate_solution_callback,
            10,
        )

        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10,
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
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10,
        )
        # 실시간 위치·GPS·고도 전체 로그는 출력하지 않는다.
        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            'Fixed-wing dynamic-gate node started. Waiting for PX4 data.'
        )
        self.get_logger().info(
            'Gate topic format: [valid, x_N, y_E, AGL, course_rad, confidence]'
        )
        self.get_logger().info(
            'Altitude plan | '
            f'WP1-through-gate={self.default_gate_agl_m:.1f}m AGL | '
            f'WP4={self.post_gate_agl_m:.1f}m AGL | '
            'control=AGL height-rate'
        )

    # ------------------------------------------------------------------
    # PX4 callbacks
    # ------------------------------------------------------------------
    def vehicle_status_callback(self, msg: VehicleStatus) -> None:
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state
        self.failsafe = msg.failsafe

    def local_position_callback(self, msg: VehicleLocalPosition) -> None:
        self.local_x = msg.x
        self.local_y = msg.y
        self.local_z = msg.z
        self.local_vx = msg.vx
        self.local_vy = msg.vy
        self.local_vz = msg.vz

        if (
            msg.dist_bottom_valid
            and math.isfinite(msg.dist_bottom)
            and msg.dist_bottom >= 0.0
        ):
            self.current_agl_m = float(msg.dist_bottom)
            self.agl_source = 'dist_bottom'
        elif math.isfinite(msg.z):
            self.current_agl_m = float(-msg.z)
            self.agl_source = 'local_z'
        else:
            self.current_agl_m = float('nan')
            self.agl_source = 'invalid'

    def airspeed_callback(self, msg: AirspeedValidated) -> None:
        """Store recent validated CAS samples for the Offboard handover."""
        sensor_valid = bool(
            getattr(msg, 'airspeed_sensor_measurement_valid', True)
        )

        calibrated = float(
            getattr(msg, 'calibrated_airspeed_m_s', float('nan'))
        )
        indicated = float(
            getattr(msg, 'indicated_airspeed_m_s', float('nan'))
        )

        if sensor_valid and math.isfinite(calibrated) and calibrated > 0.0:
            airspeed_mps = calibrated
        elif sensor_valid and math.isfinite(indicated) and indicated > 0.0:
            airspeed_mps = indicated
        else:
            self.current_airspeed_mps = float('nan')
            return

        now_sec = self.get_clock().now().nanoseconds / 1_000_000_000
        self.current_airspeed_mps = airspeed_mps
        self.last_airspeed_rx_time_sec = now_sec
        self.airspeed_samples.append((now_sec, airspeed_mps))

        cutoff_sec = now_sec - max(self.airspeed_capture_window_sec, 0.1)
        while self.airspeed_samples and self.airspeed_samples[0][0] < cutoff_sec:
            self.airspeed_samples.popleft()

    def global_position_callback(self, msg: VehicleGlobalPosition) -> None:
        self.latitude = msg.lat
        self.longitude = msg.lon
        self.altitude = msg.alt

    def command_ack_callback(self, msg: VehicleCommandAck) -> None:
        result_names = {
            0: 'ACCEPTED',
            1: 'TEMPORARILY_REJECTED',
            2: 'DENIED',
            3: 'UNSUPPORTED',
            4: 'FAILED',
            5: 'IN_PROGRESS',
            6: 'CANCELLED',
        }
        result_text = result_names.get(msg.result, f'UNKNOWN({msg.result})')
        self.get_logger().info(
            f'COMMAND ACK | command={msg.command}, result={result_text}'
        )

    def start_mission_callback(self, msg: Bool) -> None:
        if msg.data:
            self.start_mission_requested = True
            self.get_logger().info('Mission start requested.')

    # ------------------------------------------------------------------
    # Gate solution ingestion and route generation
    # ------------------------------------------------------------------
    def gate_solution_callback(self, msg: Float32MultiArray) -> None:
        # Once a route is committed, later vision jitter must not move it.
        if self.gate_route_committed or self.confirmed_gate_solution is not None:
            return

        if len(msg.data) < 6:
            self.get_logger().warning(
                'Rejected gate solution: expected 6 float values.'
            )
            return

        solution = {
            'valid': float(msg.data[0]),
            'x': float(msg.data[1]),
            'y': float(msg.data[2]),
            'agl_m': float(msg.data[3]),
            'course_rad': float(msg.data[4]),
            'confidence': float(msg.data[5]),
        }

        if not self.validate_gate_solution(solution):
            return

        if not self.gate_solution_buffer:
            self.gate_solution_buffer = [solution]
        else:
            reference = self.average_gate_solutions(self.gate_solution_buffer)
            if self.gate_solutions_consistent(reference, solution):
                self.gate_solution_buffer.append(solution)
            else:
                self.get_logger().warning(
                    'Gate solution changed beyond consistency limits; '
                    'confirmation buffer reset.'
                )
                self.gate_solution_buffer = [solution]

        # Avoid unbounded growth after the threshold is exceeded.
        self.gate_solution_buffer = self.gate_solution_buffer[
            -self.gate_solution_confirm_required:
        ]

        self.get_logger().info(
            'GATE CANDIDATE | '
            f"count={len(self.gate_solution_buffer)}/"
            f'{self.gate_solution_confirm_required} | '
            f"x={solution['x']:.1f}, y={solution['y']:.1f}, "
            f"AGL={solution['agl_m']:.1f}, "
            f"course={math.degrees(solution['course_rad']):.1f}deg, "
            f"confidence={solution['confidence']:.2f}"
        )

        if (
            len(self.gate_solution_buffer)
            >= self.gate_solution_confirm_required
        ):
            self.confirmed_gate_solution = self.average_gate_solutions(
                self.gate_solution_buffer
            )
            self.gate_route_state = 'CONFIRMED'

            self.get_logger().info(
                'GATE SOLUTION CONFIRMED | '
                f"x={self.confirmed_gate_solution['x']:.1f}, "
                f"y={self.confirmed_gate_solution['y']:.1f}, "
                f"AGL={self.confirmed_gate_solution['agl_m']:.1f}, "
                f"course={math.degrees(self.confirmed_gate_solution['course_rad']):.1f}deg"
            )

            if self.mission_state in (
                'WAIT_FOR_RC_OFFBOARD',
                'OFFBOARD_ACTIVE',
            ):
                self.commit_gate_route()

    def validate_gate_solution(self, solution: Dict[str, float]) -> bool:
        if solution['valid'] < 0.5:
            return False

        required_finite = (
            solution['x'],
            solution['y'],
            solution['agl_m'],
            solution['confidence'],
        )
        if not all(math.isfinite(value) for value in required_finite):
            self.get_logger().warning('Rejected non-finite gate solution.')
            return False

        if solution['confidence'] < self.gate_confidence_min:
            self.get_logger().warning(
                'Rejected low-confidence gate solution | '
                f"confidence={solution['confidence']:.2f}, "
                f'min={self.gate_confidence_min:.2f}'
            )
            return False

        if not (3.0 <= solution['agl_m'] <= 50.0):
            self.get_logger().warning(
                f"Rejected gate AGL outside safety range: {solution['agl_m']:.1f}m"
            )
            return False

        if math.isfinite(self.local_x) and math.isfinite(self.local_y):
            distance = math.hypot(
                solution['x'] - self.local_x,
                solution['y'] - self.local_y,
            )
            if distance > self.gate_solution_max_distance_m:
                self.get_logger().warning(
                    'Rejected gate solution beyond configured range | '
                    f'distance={distance:.1f}m'
                )
                return False

        if not math.isfinite(solution['course_rad']):
            solution['course_rad'] = self.default_gate_course_rad()
        else:
            solution['course_rad'] = self.normalize_angle(
                solution['course_rad']
            )

        return True

    def gate_solutions_consistent(
        self,
        reference: Dict[str, float],
        candidate: Dict[str, float],
    ) -> bool:
        xy_error = math.hypot(
            candidate['x'] - reference['x'],
            candidate['y'] - reference['y'],
        )
        agl_error = abs(candidate['agl_m'] - reference['agl_m'])
        course_error = abs(
            self.angle_difference(
                candidate['course_rad'],
                reference['course_rad'],
            )
        )

        return (
            xy_error <= self.gate_consistency_xy_m
            and agl_error <= self.gate_consistency_agl_m
            and course_error <= self.gate_consistency_course_rad
        )

    @staticmethod
    def average_gate_solutions(
        solutions: List[Dict[str, float]],
    ) -> Dict[str, float]:
        count = float(len(solutions))
        sin_sum = sum(math.sin(item['course_rad']) for item in solutions)
        cos_sum = sum(math.cos(item['course_rad']) for item in solutions)

        return {
            'valid': 1.0,
            'x': sum(item['x'] for item in solutions) / count,
            'y': sum(item['y'] for item in solutions) / count,
            'agl_m': sum(item['agl_m'] for item in solutions) / count,
            'course_rad': math.atan2(sin_sum, cos_sum),
            'confidence': sum(
                item['confidence'] for item in solutions
            ) / count,
        }

    def default_gate_course_rad(self) -> float:
        wp2 = next(
            item for item in self.fixed_waypoint_plan
            if item['name'] == 'WP2_RECOGNITION'
        )
        wp3 = next(
            item for item in self.fixed_waypoint_plan
            if item['name'] == 'WP3_GATE_EXIT'
        )
        return math.atan2(wp3['y'] - wp2['y'], wp3['x'] - wp2['x'])

    def build_dynamic_gate_waypoints(
        self,
        solution: Dict[str, float],
    ) -> List[Dict[str, float]]:
        course = solution['course_rad']
        direction_x = math.cos(course)
        direction_y = math.sin(course)

        gate_x = solution['x']
        gate_y = solution['y']
        gate_agl = solution['agl_m']

        approach_x = gate_x - self.gate_approach_distance_m * direction_x
        approach_y = gate_y - self.gate_approach_distance_m * direction_y
        exit_x = gate_x + self.gate_exit_distance_m * direction_x
        exit_y = gate_y + self.gate_exit_distance_m * direction_y

        return [
            self.create_waypoint(
                name='GATE_APPROACH',
                x=approach_x,
                y=approach_y,
                target_agl_m=gate_agl,
                acceptance_radius_m=6.0,
                pass_corridor_m=8.0,
                altitude_tolerance_m=3.0,
                requires_altitude=False,
                gate_course_rad=course,
            ),
            self.create_waypoint(
                name='GATE_CENTER',
                x=gate_x,
                y=gate_y,
                target_agl_m=gate_agl,
                acceptance_radius_m=self.gate_center_acceptance_radius_m,
                pass_corridor_m=self.gate_center_pass_corridor_m,
                altitude_tolerance_m=self.gate_center_altitude_tolerance_m,
                requires_altitude=False,
                gate_course_rad=course,
            ),
            self.create_waypoint(
                name='GATE_EXIT',
                x=exit_x,
                y=exit_y,
                target_agl_m=gate_agl,
                acceptance_radius_m=6.0,
                pass_corridor_m=8.0,
                altitude_tolerance_m=3.0,
                requires_altitude=False,
                gate_course_rad=course,
            ),
        ]

    def commit_gate_route(self) -> bool:
        if self.gate_route_committed:
            return True
        if self.confirmed_gate_solution is None:
            return False
        if not self.waypoint_initialized or not self.waypoints:
            return False

        current_name = self.waypoints[self.current_waypoint_index]['name']
        allowed_names = {
            'WP1_APPROACH',
            'WP2_RECOGNITION',
            'GATE_SEARCH_FORWARD',
        }
        if current_name not in allowed_names:
            self.get_logger().warning(
                f'Late gate solution ignored while tracking {current_name}.'
            )
            return False

        search_index = next(
            (
                index for index, waypoint in enumerate(self.waypoints)
                if waypoint['name'] == 'GATE_SEARCH_FORWARD'
            ),
            None,
        )
        if search_index is None:
            self.get_logger().error(
                'Cannot commit gate route: GATE_SEARCH_FORWARD not found.'
            )
            return False

        dynamic_waypoints = self.build_dynamic_gate_waypoints(
            self.confirmed_gate_solution
        )
        new_waypoints = (
            self.waypoints[:search_index]
            + dynamic_waypoints
            + self.waypoints[search_index + 1:]
        )

        # Preserve the current target when possible. If the aircraft is already
        # tracking the search waypoint, immediately retarget GATE_APPROACH.
        if current_name == 'GATE_SEARCH_FORWARD':
            new_index = search_index
        else:
            new_index = next(
                index for index, waypoint in enumerate(new_waypoints)
                if waypoint['name'] == current_name
            )

        self.waypoints = new_waypoints
        self.current_waypoint_index = new_index
        self.waypoint_reached_confirm_count = 0
        self.gate_route_committed = True
        self.gate_route_state = 'COMMITTED'
        self.load_current_waypoint_target(log_result=False)

        self.get_logger().info('DYNAMIC GATE ROUTE COMMITTED')
        for waypoint in dynamic_waypoints:
            self.get_logger().info(
                f"{waypoint['name']} | "
                f"x={waypoint['x']:.1f}, y={waypoint['y']:.1f}, "
                f"AGL={waypoint['target_agl_m']:.1f}, "
                f"AMSL={waypoint['altitude_amsl']:.1f}"
            )

        return True

    def capture_mission_airspeed(self) -> None:
        """Freeze the recent manual-flight airspeed as the mission setpoint."""
        fallback = max(
            self.minimum_mission_airspeed_mps,
            min(self.maximum_mission_airspeed_mps, self.target_airspeed_mps),
        )

        if not self.capture_manual_airspeed:
            self.mission_airspeed_mps = fallback
            self.get_logger().info(
                'AIRSPEED SET | manual capture disabled | '
                f'mission_sp={self.mission_airspeed_mps:.1f} m/s'
            )
            return

        now_sec = self.get_clock().now().nanoseconds / 1_000_000_000
        sample_is_fresh = (
            math.isfinite(self.last_airspeed_rx_time_sec)
            and now_sec - self.last_airspeed_rx_time_sec
            <= self.airspeed_sample_max_age_sec
        )

        cutoff_sec = now_sec - max(self.airspeed_capture_window_sec, 0.1)
        recent_values = [
            value
            for timestamp, value in self.airspeed_samples
            if timestamp >= cutoff_sec and math.isfinite(value) and value > 0.0
        ]

        if sample_is_fresh and recent_values:
            manual_average_mps = sum(recent_values) / len(recent_values)
            self.mission_airspeed_mps = max(
                self.minimum_mission_airspeed_mps,
                min(self.maximum_mission_airspeed_mps, manual_average_mps),
            )
            self.get_logger().info(
                'AIRSPEED CAPTURED | '
                f'manual_avg={manual_average_mps:.1f} m/s | '
                f'samples={len(recent_values)} | '
                f'mission_sp={self.mission_airspeed_mps:.1f} m/s'
            )
        else:
            self.mission_airspeed_mps = fallback
            self.get_logger().warning(
                'Airspeed capture unavailable or stale; '
                f'using fallback={self.mission_airspeed_mps:.1f} m/s'
            )

    # ------------------------------------------------------------------
    # Mission control
    # ------------------------------------------------------------------
    def control_loop(self) -> None:
        self.publish_offboard_control_mode()

        airborne_manual_ready = (
            self.arming_state == VehicleStatus.ARMING_STATE_ARMED
            and self.failsafe is False
            and math.isfinite(self.local_z)
            and self.local_z < -10.0
        )

        if self.mission_state == 'WAIT_FOR_AIRBORNE_MANUAL':
            if airborne_manual_ready:
                if not self.initialize_waypoint_list(log_result=True):
                    return
                self.offboard_warmup_count = 0
                self.mission_state = 'OFFBOARD_WARMUP'
                self.get_logger().info(
                    'State: OFFBOARD_WARMUP — preparing external setpoints.'
                )

        elif self.mission_state == 'OFFBOARD_WARMUP':
            if not self.initialize_waypoint_list(log_result=False):
                return

            self.publish_fixed_wing_setpoints()
            self.offboard_warmup_count += 1

            if self.offboard_warmup_count >= 20:
                self.mission_state = 'WAIT_FOR_RC_OFFBOARD'
                self.get_logger().info(
                    'State: WAIT_FOR_RC_OFFBOARD — select OFFBOARD using '
                    'QGC or the RC switch.'
                )

        elif self.mission_state == 'WAIT_FOR_RC_OFFBOARD':
            self.publish_fixed_wing_setpoints()

            if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                if not self.finalize_ground_reference():
                    return

                self.offboard_entry_agl_m = self.current_agl_m
                self.capture_mission_airspeed()

                if not self.initialize_waypoint_list(log_result=True):
                    self.get_logger().error('Failed to finalize waypoint list.')
                    return

                if self.confirmed_gate_solution is not None:
                    self.commit_gate_route()

                self.current_waypoint_index = 0
                if not self.load_current_waypoint_target():
                    self.get_logger().error('Failed to load first waypoint.')
                    return

                self.offboard_active_start_time = self.get_clock().now()
                self.mission_state = 'OFFBOARD_ACTIVE'
                self.get_logger().info(
                    'State: OFFBOARD_ACTIVE — Offboard activated externally.'
                )

        elif self.mission_state == 'OFFBOARD_ACTIVE':
            if self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.mission_state = 'COMPLETE'
                self.get_logger().warning(
                    'State: COMPLETE — pilot exited Offboard externally.'
                )
                return

            if self.failsafe is True:
                self.get_logger().error(
                    'Failsafe detected. Waiting for PX4 or pilot takeover.'
                )
                self.mission_state = 'WAIT_FOR_PILOT_TAKEOVER'
                return

            if (
                self.confirmed_gate_solution is not None
                and not self.gate_route_committed
            ):
                self.commit_gate_route()

            self.publish_fixed_wing_setpoints()
            self.track_current_waypoint()

        elif self.mission_state == 'WAIT_FOR_PILOT_TAKEOVER':
            if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.publish_fixed_wing_setpoints()
            else:
                self.mission_state = 'COMPLETE'
                self.get_logger().info(
                    'State: COMPLETE — pilot control restored.'
                )

        elif self.mission_state == 'COMPLETE':
            pass

    def track_current_waypoint(self) -> None:
        if not self.waypoints:
            return

        current = self.waypoints[self.current_waypoint_index]
        distance_m = self.distance_to_current_waypoint()

        previous_x, previous_y = self.previous_path_point()
        passed_plane, cross_track_m = self.waypoint_pass_geometry(
            previous_x,
            previous_y,
            current['x'],
            current['y'],
        )

        is_gate_center = current['name'] == 'GATE_CENTER'

        altitude_error_m = abs(
            self.current_agl_m - current['target_agl_m']
        )
        altitude_ok = (
            not current['requires_altitude']
            or (
                math.isfinite(altitude_error_m)
                and altitude_error_m <= current['altitude_tolerance_m']
            )
        )
        inside_pass_corridor = (
            math.isfinite(cross_track_m)
            and cross_track_m <= current['pass_corridor_m']
        )

        # A gate-plane crossing is terminal: accept a centered crossing, but
        # never command a fixed-wing aircraft to turn back toward a missed
        # gate that is already behind it.
        if is_gate_center and passed_plane:
            if inside_pass_corridor and altitude_ok:
                self.waypoint_reached_confirm_count += 1
            else:
                self.gate_route_state = 'MISSED'
                self.get_logger().error(
                    'GATE MISSED | '
                    f'cross_track={cross_track_m:.1f}m '
                    f'(limit={current["pass_corridor_m"]:.1f}m) | '
                    f'altitude_error={altitude_error_m:.1f}m '
                    f'(limit={current["altitude_tolerance_m"]:.1f}m). '
                    'Pilot takeover required.'
                )
                self.mission_state = 'WAIT_FOR_PILOT_TAKEOVER'
                return

        waypoint_condition_met = (
            not (is_gate_center and passed_plane)
            and (
                distance_m <= current['acceptance_radius_m']
                or (passed_plane and inside_pass_corridor)
            )
            and altitude_ok
        )

        if self.debug_log_count % 30 == 0:
            self.get_logger().info(
                f"WP APPROACH | {current['name']} | "
                f"target=({current['x']:.1f}, {current['y']:.1f}) | "
                f"airspeed_sp={self.mission_airspeed_mps:.1f} m/s"
            )
        
        self.debug_log_count += 1

        if waypoint_condition_met:
            self.waypoint_reached_confirm_count += 1

        elif not (is_gate_center and passed_plane):
            self.waypoint_reached_confirm_count = 0

        if (
            self.waypoint_reached_confirm_count
            >= self.waypoint_reached_confirm_required
        ):
            reached_name = current['name']

            self.get_logger().info(
                f"WP PASSED | {reached_name} | "
                f"target=({current['x']:.1f}, {current['y']:.1f}) | "
                f"airspeed_sp={self.mission_airspeed_mps:.1f} m/s"
            )

            if reached_name == 'GATE_SEARCH_FORWARD' and not self.gate_route_committed:
                self.get_logger().error(
                    'No valid gate route before end of search segment. '
                    'Pilot takeover required.'
                )
                self.mission_state = 'WAIT_FOR_PILOT_TAKEOVER'
                return

            if reached_name == 'GATE_CENTER':
                self.gate_route_state = 'PASSED'

            if self.advance_to_next_waypoint():
                return

            self.get_logger().info(
                f'All {len(self.waypoints)} waypoints completed. '
                'Switch to POSITION, HOLD, or STABILIZED manually.'
            )
            self.mission_state = 'WAIT_FOR_PILOT_TAKEOVER'
            return

        if self.offboard_active_start_time is not None:
            elapsed_sec = (
                self.get_clock().now() - self.offboard_active_start_time
            ).nanoseconds / 1_000_000_000
            if elapsed_sec >= self.offboard_test_timeout_sec:
                self.get_logger().warning(
                    'Waypoint mission timeout. Pilot takeover required.'
                )
                self.mission_state = 'WAIT_FOR_PILOT_TAKEOVER'

    # ------------------------------------------------------------------
    # Route/altitude helpers
    # ------------------------------------------------------------------
    def finalize_ground_reference(self) -> bool:
        if not all(
            math.isfinite(value)
            for value in (self.current_agl_m, self.altitude, self.local_z)
        ):
            self.get_logger().error(
                'Cannot finalize route: AGL, AMSL, or local z is invalid.'
            )
            return False

        self.ground_altitude_amsl = self.altitude - self.current_agl_m
        self.ground_local_z = self.local_z + self.current_agl_m

        self.get_logger().info(
            'GROUND REFERENCE | '
            f'current_AMSL={self.altitude:.2f}m | '
            f'current_AGL={self.current_agl_m:.2f}m | '
            f'ground_AMSL={self.ground_altitude_amsl:.2f}m | '
            f'ground_local_z={self.ground_local_z:.2f}m'
        )
        return True

    def initialize_waypoint_list(self, *, log_result: bool = True) -> bool:
        if not all(
            math.isfinite(value)
            for value in (
                self.local_x,
                self.local_y,
                self.local_z,
                self.altitude,
                self.current_agl_m,
            )
        ):
            if log_result:
                self.get_logger().warning(
                    'Cannot initialize route: position or altitude invalid.'
                )
            return False

        self.route_start_x = self.local_x
        self.route_start_y = self.local_y

        if not (
            math.isfinite(self.ground_altitude_amsl)
            and math.isfinite(self.ground_local_z)
        ):
            self.ground_altitude_amsl = self.altitude - self.current_agl_m
            self.ground_local_z = self.local_z + self.current_agl_m

        cruise_agl_m = (
            self.offboard_entry_agl_m
            if math.isfinite(self.offboard_entry_agl_m)
            else self.current_agl_m
        )
        cruise_agl_m = max(5.0, float(cruise_agl_m))

        self.waypoints = [
            self.create_waypoint(
                name=plan['name'],
                x=float(plan['x']),
                y=float(plan['y']),
                target_agl_m=float(
                    plan.get('target_agl_m', cruise_agl_m)
                ),
                acceptance_radius_m=self.waypoint_acceptance_radius_m,
                pass_corridor_m=self.waypoint_pass_corridor_m,
                altitude_tolerance_m=self.waypoint_altitude_acceptance_m,
                # Base-route waypoints must never hold a fixed-wing aircraft
                # in an orbit while waiting for altitude convergence. Altitude is
                # still commanded continuously, but only GATE_CENTER performs a
                # strict altitude acceptance check.
                requires_altitude=False,
            )
            for plan in self.fixed_waypoint_plan
        ]

        self.current_waypoint_index = 0
        self.waypoint_reached_confirm_count = 0
        self.waypoint_initialized = True
        self.gate_route_committed = False
        self.gate_route_state = (
            'CONFIRMED' if self.confirmed_gate_solution is not None else 'WAITING'
        )
        self.load_current_waypoint_target(log_result=False)

        if log_result:
            self.get_logger().info('Base WP route initialized.')
            for waypoint in self.waypoints:
                self.get_logger().info(
                    f"{waypoint['name']} | "
                    f"x={waypoint['x']:.1f}, y={waypoint['y']:.1f}, "
                    f"AGL={waypoint['target_agl_m']:.1f}, "
                    f"AMSL={waypoint['altitude_amsl']:.1f}"
                )
        return True

    def create_waypoint(
        self,
        *,
        name: str,
        x: float,
        y: float,
        target_agl_m: float,
        acceptance_radius_m: float,
        pass_corridor_m: float,
        altitude_tolerance_m: float,
        requires_altitude: bool,
        gate_course_rad: float = float('nan'),
    ) -> Dict[str, float]:
        target_agl_m = max(3.0, float(target_agl_m))
        altitude_amsl = self.ground_altitude_amsl + target_agl_m
        local_z = self.ground_local_z - target_agl_m

        return {
            'name': name,
            'x': float(x),
            'y': float(y),
            'z': float(local_z),
            'altitude_amsl': float(altitude_amsl),
            'target_agl_m': float(target_agl_m),
            'acceptance_radius_m': float(acceptance_radius_m),
            'pass_corridor_m': float(pass_corridor_m),
            'altitude_tolerance_m': float(altitude_tolerance_m),
            'requires_altitude': bool(requires_altitude),
            'gate_course_rad': float(gate_course_rad),
        }

    def load_current_waypoint_target(
        self,
        *,
        log_result: bool = True,
    ) -> bool:
        if (
            not self.waypoints
            or self.current_waypoint_index >= len(self.waypoints)
        ):
            return False

        waypoint = self.waypoints[self.current_waypoint_index]
        self.target_x = waypoint['x']
        self.target_y = waypoint['y']
        self.target_z = waypoint['z']
        self.hold_altitude_amsl = waypoint['altitude_amsl']
        self.hold_agl_m = waypoint['target_agl_m']

        if log_result:
            self.get_logger().info(
                f"WP ENTER | {waypoint['name']} | "
                f"target=({self.target_x:.1f}, {self.target_y:.1f}) | "
                f"airspeed_sp={self.mission_airspeed_mps:.1f} m/s"
            )
        return True

    def advance_to_next_waypoint(self) -> bool:
        next_index = self.current_waypoint_index + 1
    
        if next_index >= len(self.waypoints):
            return False
    
        self.current_waypoint_index = next_index
        self.waypoint_reached_confirm_count = 0
    
        # 새 WP 진입 시 접근 로그 주기를 처음부터 시작한다.
        self.debug_log_count = 0
    
        return self.load_current_waypoint_target(log_result=True)

    def previous_path_point(self) -> tuple:
        if self.current_waypoint_index == 0:
            return self.route_start_x, self.route_start_y
        previous = self.waypoints[self.current_waypoint_index - 1]
        return previous['x'], previous['y']

    def waypoint_pass_geometry(
        self,
        previous_x: float,
        previous_y: float,
        waypoint_x: float,
        waypoint_y: float,
    ) -> tuple:
        path_dx = waypoint_x - previous_x
        path_dy = waypoint_y - previous_y
        path_length = math.hypot(path_dx, path_dy)
        if path_length < 1.0:
            return False, float('inf')

        unit_x = path_dx / path_length
        unit_y = path_dy / path_length
        aircraft_dx = self.local_x - waypoint_x
        aircraft_dy = self.local_y - waypoint_y

        passed_plane = aircraft_dx * unit_x + aircraft_dy * unit_y >= 0.0
        cross_track_m = abs(aircraft_dx * unit_y - aircraft_dy * unit_x)
        return passed_plane, cross_track_m

    # ------------------------------------------------------------------
    # PX4 setpoints
    # ------------------------------------------------------------------
    def publish_offboard_control_mode(self) -> None:
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        self.offboard_control_mode_pub.publish(msg)

    def publish_fixed_wing_setpoints(self) -> None:
        if (
            not self.waypoint_initialized
            or not all(
                math.isfinite(value)
                for value in (
                    self.local_x,
                    self.local_y,
                    self.local_vx,
                    self.local_vy,
                    self.current_agl_m,
                    self.hold_agl_m,
                )
            )
        ):
            return

        dx = self.target_x - self.local_x
        dy = self.target_y - self.local_y
        distance_m = math.hypot(dx, dy)

        if distance_m >= 1.0:
            target_course_rad = math.atan2(dy, dx)
        else:
            target_course_rad = self.get_current_course()
            if not math.isfinite(target_course_rad):
                return

        timestamp_us = int(self.get_clock().now().nanoseconds / 1000)

        lateral_msg = FixedWingLateralSetpoint()
        lateral_msg.timestamp = timestamp_us
        lateral_msg.course = float(target_course_rad)
        lateral_msg.airspeed_direction = float('nan')
        lateral_msg.lateral_acceleration = float('nan')

        # Use a direct height-rate setpoint derived from AGL error.
        # PX4 defines positive height_rate as climbing (ENU Up positive).
        # This avoids any AMSL/geoid-reference mismatch in simulation and
        # gives an explicit descent command when current AGL is too high.
        agl_error_m = self.hold_agl_m - self.current_agl_m

        if abs(agl_error_m) <= self.altitude_deadband_m:
            height_rate_cmd = 0.0
        else:
            height_rate_cmd = self.altitude_rate_kp * agl_error_m
            height_rate_cmd = max(
                -self.max_sink_rate_mps,
                min(self.max_climb_rate_mps, height_rate_cmd),
            )

        self.commanded_height_rate_mps = float(height_rate_cmd)

        longitudinal_msg = FixedWingLongitudinalSetpoint()
        longitudinal_msg.timestamp = timestamp_us
        longitudinal_msg.altitude = float('nan')
        longitudinal_msg.height_rate = float(height_rate_cmd)
        longitudinal_msg.equivalent_airspeed = float(self.mission_airspeed_mps)
        longitudinal_msg.pitch_direct = float('nan')
        longitudinal_msg.throttle_direct = float('nan')

        self.fw_lateral_setpoint_pub.publish(lateral_msg)
        self.fw_longitudinal_setpoint_pub.publish(longitudinal_msg)

    def publish_trajectory_setpoint(self) -> None:
        if not self.waypoint_initialized:
            return

        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = [self.target_x, self.target_y, self.target_z]
        msg.velocity = [float('nan')] * 3
        msg.acceleration = [float('nan')] * 3
        msg.jerk = [float('nan')] * 3
        msg.yaw = float('nan')
        msg.yawspeed = float('nan')
        self.trajectory_setpoint_pub.publish(msg)

    def publish_vehicle_command(
        self,
        command: int,
        *,
        param1: float = 0.0,
        param2: float = 0.0,
    ) -> None:
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)

    # ------------------------------------------------------------------
    # Utility/logging
    # ------------------------------------------------------------------
    def distance_to_current_waypoint(self) -> float:
        if not self.waypoint_initialized or not self.waypoints:
            return float('inf')
        return math.hypot(
            self.target_x - self.local_x,
            self.target_y - self.local_y,
        )

    def get_current_course(self) -> float:
        speed_horizontal = math.hypot(self.local_vx, self.local_vy)
        if speed_horizontal < 1.0:
            return float('nan')
        return math.atan2(self.local_vy, self.local_vx)

    @staticmethod
    def normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    @classmethod
    def angle_difference(cls, first: float, second: float) -> float:
        return cls.normalize_angle(first - second)

    def print_vehicle_state(self) -> None:
        current_name = (
            self.waypoints[self.current_waypoint_index]['name']
            if self.waypoints
            and self.current_waypoint_index < len(self.waypoints)
            else 'none'
        )
        self.get_logger().info(
            '\n'
            f'nav_state      : {self.nav_state}\n'
            f'arming_state   : {self.arming_state}\n'
            f'failsafe       : {self.failsafe}\n'
            f'mission_state  : {self.mission_state}\n'
            f'gate_state     : {self.gate_route_state}\n'
            f'current_target : {current_name}\n'
            f'local NED      : x={self.local_x:.2f}, '
            f'y={self.local_y:.2f}, z={self.local_z:.2f}\n'
            f'height AGL     : {self.current_agl_m:.2f}m '
            f'[{self.agl_source}]\n'
            f'vertical speed : {self.local_vz:.2f}m/s [Down positive]\n'
            f'global GPS     : lat={self.latitude:.7f}, '
            f'lon={self.longitude:.7f}, '
            f'alt_AMSL={self.altitude:.2f}m'
        )


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
