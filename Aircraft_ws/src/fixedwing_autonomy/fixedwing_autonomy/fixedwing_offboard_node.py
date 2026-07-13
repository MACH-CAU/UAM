#!/usr/bin/env python3

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
)


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

        # 현재 GPS 위치 저장
        self.latitude = float('nan')
        self.longitude = float('nan')
        self.altitude = float('nan')

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

        # 1초마다 현재 상태 출력
        self.print_timer = self.create_timer(
            1.0,
            self.print_vehicle_state,
        )

        self.get_logger().info(
            'Fixed-wing offboard node started. Waiting for PX4 data...'
        )

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