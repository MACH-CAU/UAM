import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge
import cv2
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy


class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher') # Node name
        
        qos_profile = QoSProfile(
           reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,  # UDP 기반, 손실 감수
           durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,  # 최신 메시지만 유지 (구독 전 메시지 받을 필요 없음)
           history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,  # 최신 데이터 유지
           depth=100  # 최대한 많은 메시지 유지
        )
        
        self.publisher_=self.create_publisher(
            Image, # Message type
            'video_frames', # Topic name
            qos_profile)

        timer_period=0.01
        self.timer=self.create_timer(timer_period, self.timer_callback)

        self.cap=cv2.VideoCapture(0)

        self.br=CvBridge()

    def timer_callback(self):
        ret, frame=self.cap.read()

        if ret==True:
            self.publisher_.publish(self.br.cv2_to_imgmsg(frame))
            self.get_logger().info('Publishing video frame')

def main():
    rclpy.init() # ROS 통신 시작
    image_publisher=ImagePublisher() # 클래스 생성
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()

    rclpy.shutdown() # ROS 통신 종료

if __name__=='__main__':
    main()
