import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber') # Node name
        self.subscription=self.create_subscription(
            Image, # Message type
            'video_frames', # Topic name
            self.listener_callback,
            100)
        self.subscription
        self.br=CvBridge()

    def listener_callback(self, data):

        self.get_logger().info('Receiving Video frame')

        current_frame=self.br.imgmsg_to_cv2(data)

        cv2.imshow("camera",current_frame)
        cv2.waitKey(1)

def main():

    rclpy.init() # ROS 통신 시작
    image_subscriber=ImageSubscriber() # 클래스 생성
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()

    rclpy.shutdown() # ROS 통신 종료


if __name__=='__main__':
    main()