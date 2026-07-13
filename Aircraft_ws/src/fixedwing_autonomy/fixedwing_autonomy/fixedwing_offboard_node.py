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
    VehicleCommand,
    VehicleCommandAck,
    OffboardControlMode,
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
        
        self.command_ack_received = False
        self.test_start_time = self.get_clock().now()
        self.offboard_counter = 0
        self.offboard_command_sent = False
        
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

        # 1초마다 현재 상태 출력
        self.print_timer = self.create_timer(
            1.0,
            self.print_vehicle_state,
        )

        self.get_logger().info(
            'Fixed-wing offboard node started. Waiting for PX4 data...'
        )
        
        self.offboard_timer = self.create_timer(
            0.1,
            self.offboard_test_callback,
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
    
    def offboard_test_callback(self) -> None:
        # Offboard heartbeat는 계속 보냄
        self.publish_offboard_control_mode()
    
        self.offboard_counter += 1
    
        # 10 Hz 기준 20회 = 약 2초
        if (
            self.offboard_counter >= 20
            and not self.offboard_command_sent
        ):
            self.request_offboard_mode()
            self.offboard_command_sent = True

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