import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher') # Node name
        self.publisher_=self.create_publisher(
            Image, # Message type
            'video_frames', # Topic name
            100)
        timer_period=0.1
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
