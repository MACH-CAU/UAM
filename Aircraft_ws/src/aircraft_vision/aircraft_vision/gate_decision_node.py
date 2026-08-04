#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String


class GateDecisionNode(Node):
    """
    선택된 이미지 종류를 시험용 게이트 좌표로 변환한다.

    입력:
        /selected_gate
        std_msgs/msg/String
        "tank" 또는 "building"

    출력:
        /vision/gate_solution_ned
        std_msgs/msg/Float32MultiArray

        [
            valid,
            gate_x_north_m,
            gate_y_east_m,
            gate_agl_m,
            gate_course_rad,
            confidence
        ]
    """

    def __init__(self) -> None:
        super().__init__('gate_decision_node')

        # -------------------------------------------------
        # 시험용 게이트 좌표
        #
        # 현재 기본 비행 방향은 +y(East) 방향이다.
        # +x는 진행 방향 기준 오른쪽,
        # -x는 진행 방향 기준 왼쪽이다.
        # -------------------------------------------------
        self.declare_parameter('tank_gate_x_m', -5.0)
        self.declare_parameter('tank_gate_y_m', 325.0)

        self.declare_parameter('building_gate_x_m', 5.0)
        self.declare_parameter('building_gate_y_m', 325.0)

        self.declare_parameter('gate_agl_m', 6.0)
        self.declare_parameter('gate_course_deg', 90.0)
        self.declare_parameter('confidence', 0.95)

        self.tank_gate_x_m = float(
            self.get_parameter('tank_gate_x_m').value
        )
        self.tank_gate_y_m = float(
            self.get_parameter('tank_gate_y_m').value
        )

        self.building_gate_x_m = float(
            self.get_parameter('building_gate_x_m').value
        )
        self.building_gate_y_m = float(
            self.get_parameter('building_gate_y_m').value
        )

        self.gate_agl_m = float(
            self.get_parameter('gate_agl_m').value
        )
        self.gate_course_deg = float(
            self.get_parameter('gate_course_deg').value
        )
        self.confidence = float(
            self.get_parameter('confidence').value
        )

        # 같은 선택 결과가 반복되어도 한 번만 확정한다.
        self.selected_gate = None
        self.selection_locked = False

        self.gate_subscriber = self.create_subscription(
            String,
            '/selected_gate',
            self.selected_gate_callback,
            10,
        )

        self.solution_publisher = self.create_publisher(
            Float32MultiArray,
            '/vision/gate_solution_ned',
            10,
        )

        self.get_logger().info(
            'Gate decision node started | '
            'waiting for /selected_gate: tank or building'
        )

        self.get_logger().info(
            'Test gate map | '
            f'tank=({self.tank_gate_x_m:.1f}, '
            f'{self.tank_gate_y_m:.1f}) | '
            f'building=({self.building_gate_x_m:.1f}, '
            f'{self.building_gate_y_m:.1f})'
        )

    def selected_gate_callback(self, message: String) -> None:
        gate_name = message.data.strip().lower()

        if self.selection_locked:
            if gate_name != self.selected_gate:
                self.get_logger().warning(
                    'Gate selection already locked | '
                    f'current={self.selected_gate}, '
                    f'ignored={gate_name}'
                )
            return

        if gate_name == 'tank':
            gate_x = self.tank_gate_x_m
            gate_y = self.tank_gate_y_m

        elif gate_name == 'building':
            gate_x = self.building_gate_x_m
            gate_y = self.building_gate_y_m

        else:
            self.get_logger().warning(
                f'Unknown gate selection: {gate_name}'
            )
            return

        self.selected_gate = gate_name
        self.selection_locked = True

        self.publish_gate_solution(
            gate_name=gate_name,
            gate_x=gate_x,
            gate_y=gate_y,
        )

    def publish_gate_solution(
        self,
        *,
        gate_name: str,
        gate_x: float,
        gate_y: float,
    ) -> None:
        course_rad = math.radians(self.gate_course_deg)

        for _ in range(3):
            message = Float32MultiArray()
        
            message.data = [
                1.0,
                float(gate_x),
                float(gate_y),
                float(self.gate_agl_m),
                float(course_rad),
                float(self.confidence),
            ]
        
            self.solution_publisher.publish(message)
        
        self.get_logger().info(
            f'GATE SELECTED | '
            f'{gate_name.upper()} | '
            f'target=({gate_x:.1f}, {gate_y:.1f})'
        )


def main(args=None) -> None:
    rclpy.init(args=args)

    node = GateDecisionNode()

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