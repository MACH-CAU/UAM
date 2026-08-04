#!/usr/bin/env python3

import json
import time
from typing import Any

import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class GateDetectorNode(Node):

    def __init__(self):
        super().__init__('gate_detector_node')

        # -----------------------------
        # ROS 2 parameters
        # -----------------------------
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('confidence_threshold', 0.60)
        self.declare_parameter('publish_fps', 10.0)
        self.declare_parameter(
            'engine_path',
            '/home/jetson/aircraft_vision/models/yolox_tiny_fp16.engine'
        )

        self.camera_index = int(
            self.get_parameter('camera_index').value
        )

        self.confidence_threshold = float(
            self.get_parameter('confidence_threshold').value
        )

        self.publish_fps = float(
            self.get_parameter('publish_fps').value
        )

        self.engine_path = str(
            self.get_parameter('engine_path').value
        )

        # -----------------------------
        # Publisher
        # -----------------------------
        self.detection_pub = self.create_publisher(
            String,
            '/gate_detection',
            10
        )

        # -----------------------------
        # Camera
        # -----------------------------
        self.camera = cv2.VideoCapture(self.camera_index)

        if not self.camera.isOpened():
            raise RuntimeError(
                f'카메라를 열 수 없습니다: index={self.camera_index}'
            )

        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # -----------------------------
        # YOLO TensorRT initialization
        # -----------------------------
        self.detector = self.initialize_detector()

        # -----------------------------
        # Timer
        # -----------------------------
        timer_period = 1.0 / max(self.publish_fps, 1.0)

        self.timer = self.create_timer(
            timer_period,
            self.timer_callback
        )

        self.get_logger().info('Gate detector node started')
        self.get_logger().info(
            f'Engine path: {self.engine_path}'
        )
        self.get_logger().info(
            f'Confidence threshold: {self.confidence_threshold:.2f}'
        )

    def initialize_detector(self) -> Any:
        """
        기존 YOLOX TensorRT 추론 객체를 생성하는 부분.

        네 test_yolox_trt.py에서 사용하는 TensorRT 클래스에 맞게
        이 함수 내부만 수정하면 된다.
        """

        try:
            # 예시:
            #
            # from aircraft_vision.yolox_trt import YOLOXTRT
            # detector = YOLOXTRT(self.engine_path)
            # return detector

            self.get_logger().warning(
                'YOLO TensorRT 객체가 아직 연결되지 않았습니다. '
                'initialize_detector()를 기존 추론 코드에 맞게 수정해야 합니다.'
            )

            return None

        except Exception as error:
            raise RuntimeError(
                f'YOLO TensorRT 초기화 실패: {error}'
            ) from error

    def timer_callback(self):
        success, frame = self.camera.read()

        if not success or frame is None:
            self.get_logger().warning('카메라 프레임 수신 실패')
            return

        try:
            detections = self.run_inference(frame)

        except Exception as error:
            self.get_logger().error(
                f'YOLO 추론 실패: {error}'
            )
            return

        best_detection = self.select_best_detection(detections)

        message = String()

        if best_detection is None:
            message.data = json.dumps(
                {
                    'timestamp': time.time(),
                    'valid': False,
                    'class_name': 'unknown',
                    'confidence': 0.0,
                    'center_x': 0.0,
                    'center_y': 0.0,
                    'width': 0.0,
                    'height': 0.0
                }
            )

        else:
            message.data = json.dumps(
                {
                    'timestamp': time.time(),
                    'valid': True,
                    'class_name': best_detection['class_name'],
                    'confidence': best_detection['confidence'],
                    'center_x': best_detection['center_x'],
                    'center_y': best_detection['center_y'],
                    'width': best_detection['width'],
                    'height': best_detection['height']
                }
            )

        self.detection_pub.publish(message)

    def run_inference(self, frame) -> list[dict]:
        """
        기존 YOLOX TensorRT 추론 코드와 연결하는 함수.

        반환 형식:

        [
            {
                'class_name': 'tank',
                'confidence': 0.91,
                'x1': 100.0,
                'y1': 80.0,
                'x2': 300.0,
                'y2': 350.0
            }
        ]
        """

        if self.detector is None:
            # 컴패니언 컴퓨터가 없는 현재 개발 단계에서는
            # 검출 없음 상태를 발행하도록 한다.
            return []

        # 기존 추론 코드에 맞게 이 부분을 수정한다.
        #
        # 예시:
        #
        # raw_detections = self.detector.infer(frame)
        # detections = []
        #
        # for detection in raw_detections:
        #     detections.append({
        #         'class_name': detection['class_name'],
        #         'confidence': float(detection['confidence']),
        #         'x1': float(detection['x1']),
        #         'y1': float(detection['y1']),
        #         'x2': float(detection['x2']),
        #         'y2': float(detection['y2'])
        #     })
        #
        # return detections

        return []

    def select_best_detection(
        self,
        detections: list[dict]
    ) -> dict | None:

        valid_classes = {'tank', 'building'}
        best_detection = None

        for detection in detections:

            class_name = str(
                detection.get('class_name', '')
            ).lower()

            confidence = float(
                detection.get('confidence', 0.0)
            )

            if class_name not in valid_classes:
                continue

            if confidence < self.confidence_threshold:
                continue

            x1 = float(detection['x1'])
            y1 = float(detection['y1'])
            x2 = float(detection['x2'])
            y2 = float(detection['y2'])

            if x2 <= x1 or y2 <= y1:
                continue

            converted_detection = {
                'class_name': class_name,
                'confidence': confidence,
                'center_x': (x1 + x2) / 2.0,
                'center_y': (y1 + y2) / 2.0,
                'width': x2 - x1,
                'height': y2 - y1
            }

            if best_detection is None:
                best_detection = converted_detection

            elif confidence > best_detection['confidence']:
                best_detection = converted_detection

        return best_detection

    def destroy_node(self):
        if self.camera is not None:
            self.camera.release()

        cv2.destroyAllWindows()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = GateDetectorNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as error:
        print(f'Gate detector node error: {error}')

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()