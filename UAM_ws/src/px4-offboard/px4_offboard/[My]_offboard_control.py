""" Take off Test by Haechan """
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint, VehicleStatus

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        # 퍼블리셔 생성 (PX4로 명령 보내기)
        self.command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.offboard_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)

        # 서브스크라이버 생성 (드론 상태 확인)
        self.status_subscriber = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.status_callback, qos_profile)

        # 드론 상태 변수
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        
        self.state = "INIT"
        # 타이머 설정 (20ms마다 명령 전송)
        self.timer = self.create_timer(0.02, self.offboard_control_loop)

        # 실행 단계 관리

    def status_callback(self, msg):
        """드론 상태 업데이트 (NAV 상태 & ARM 상태)"""
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def send_vehicle_command(self, command, param1=0.0, param2=0.0):
        """PX4에 VehicleCommand 메시지를 전송하는 함수"""
        msg = VehicleCommand()
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_publisher.publish(msg)
        self.get_logger().info(f"Sent command: {command}, param1: {param1}, param2: {param2}")

    def offboard_control_loop(self):
        """드론의 상태에 따라 Offboard, Arm, Takeoff 실행"""
  
    
        # if self.state == "INIT":
        
        #     if self.arming_state != VehicleStatus.ARMING_STATE_ARMED:
        #         self.get_logger().info("Switching to OFFBOARD mode...")
        #         self.send_vehicle_command(176, 1.0)  # VEHICLE_CMD_DO_SET_MODE (Offboard 모드 설정)
        #         self.state = "ARM"
        #     else :
        #         self.send_vehicle_command(400, 0.0) 

        # elif self.state == "ARM" and self.arming_state != VehicleStatus.ARMING_STATE_ARMED:
        #     self.get_logger().info("Arming the drone...")
        #     self.send_vehicle_command(400, 1.0)  # VEHICLE_CMD_COMPONENT_ARM_DISARM (Arm)
        # elif self.state == "ARM" and self.arming_state == VehicleStatus.ARMING_STATE_ARMED:
        #     self.state = "TAKEOFF"
        # elif self.state == "TAKEOFF" and self.arming_state == VehicleStatus.ARMING_STATE_ARMED:
        #     self.get_logger().info("Taking off...")
            
        #     # self.send_vehicle_command(22, 5.0)  # VEHICLE_CMD_COMPONENT_ARM_DISARM (Arm)
        #     # takeoff_msg.position[0] = 0.0  # X (위치)
        #     # takeoff_msg.position[1] = 0.0  # Y (위치)
        #     # takeoff_msg.position[2] = -5.0  # Z (고도, PX4는 NED 좌표계라 -값)
        #     # self.trajectory_publisher.publish(takeoff_msg)
        #     # self.state = "HOLD"
            
            

        # elif self.state == "HOLD":
        #     self.get_logger().info("Holding position at 5m altitude.")
        offboard_msg = OffboardControlMode()
        offboard_msg.position=True
        offboard_msg.velocity=False
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.acceleration=False
        self.offboard_publisher.publish(offboard_msg)
        if  (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.arming_state == VehicleStatus.ARMING_STATE_ARMED):

            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.position[0] = 0#self.radius * np.cos(self.theta)
            trajectory_msg.position[1] = 0#self.radius * np.sin(self.theta)
            trajectory_msg.position[2] = -self.altitude
            self.publisher_trajectory.publish(trajectory_msg)
def main(args=None):
    rclpy.init(args=args)
    drone_controller = DroneController()
    rclpy.spin(drone_controller)
    drone_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
