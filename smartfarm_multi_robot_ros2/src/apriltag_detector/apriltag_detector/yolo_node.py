import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch
from ultralytics import YOLO
from std_msgs.msg import String

class YoloObjectDetector(Node):
    def __init__(self):
        super().__init__('yolo_object_detector')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        
        # YOLOv8 모델 로드 (Ultralytics 제공 사전 학습 모델)
        self.model = YOLO("yolov8m.pt")  # YOLOv8 모델을 사용 (필요에 따라 모델 변경 가능)
        
        # 감지할 클래스 설정 (apple, orange, banana)
        self.target_classes = {'apple', 'orange', 'banana'}  # 원하는 클래스만 필터링
        
        # 스마트팜 UI에 알릴 수 있도록 퍼블리셔 추가
        self.detected_object_publisher = self.create_publisher(String, '/detected_object', 10)
        
        # OpenCV 창을 초기화
        cv2.namedWindow("YOLO Object Detector", cv2.WINDOW_NORMAL)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # YOLO를 사용하여 객체 감지
        results = self.model(cv_image)
        
        detected_labels = set()  # 감지된 라벨 집합
        
        for result in results:
            for box in result.boxes:
                cls = int(box.cls[0].item())  # 클래스 ID
                label = self.model.names[cls]  # 클래스 이름
                
                if label in self.target_classes:
                    detected_labels.add(label)  # 감지된 라벨 집합에 추가
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())  # 바운딩 박스 좌표
                    confidence = box.conf[0].item()  # 신뢰도
                    
                    # 바운딩 박스 및 텍스트 출력
                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(cv_image, f"{label} {confidence:.2f}", (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    self.get_logger().info(f"Detected: {label}, Confidence: {confidence:.2f}")
        
        # 감지된 객체가 있으면 UI에 전달, 없으면 "None" 전달
        msg = String()
        if detected_labels:
            detected_objects = ', '.join(detected_labels)
            msg.data = f"{detected_objects}"  # 감지된 객체 목록 전달
        else:
            msg.data = "None"  # 감지된 객체가 없을 경우 "None" 전달
        
        self.detected_object_publisher.publish(msg)
        
        # OpenCV 창을 업데이트
        cv2.imshow("YOLO Object Detector", cv_image)

def main(args=None):
    rclpy.init(args=args)
    node = YoloObjectDetector()

    # ROS2 콜백 루프 시작 (spin_once 사용)
    while rclpy.ok():
        rclpy.spin_once(node)  # 한 번의 콜백 처리
        cv2.waitKey(1)  # OpenCV 이벤트 처리

    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch
from ultralytics import YOLO
from std_msgs.msg import String

class YoloObjectDetector(Node):
    def __init__(self):
        super().__init__('yolo_object_detector')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        
        # YOLOv8 모델 로드 (Ultralytics 제공 사전 학습 모델)
        self.model = YOLO("yolov8m.pt")  # YOLOv8 모델을 사용 (필요에 따라 모델 변경 가능)
        
        # 감지할 클래스 설정 (apple, orange, banana)
        self.target_classes = {'apple', 'orange', 'banana'}  # 원하는 클래스만 필터링
        
        # 스마트팜 UI에 알릴 수 있도록 퍼블리셔 추가
        self.detected_object_publisher = self.create_publisher(String, '/detected_object', 10)
        
        # OpenCV 창을 초기화
        cv2.namedWindow("YOLO Object Detector", cv2.WINDOW_NORMAL)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # YOLO를 사용하여 객체 감지
        results = self.model(cv_image)
        
        detected_labels = set()  # 감지된 라벨 집합
        
        for result in results:
            for box in result.boxes:
                cls = int(box.cls[0].item())  # 클래스 ID
                label = self.model.names[cls]  # 클래스 이름
                
                if label in self.target_classes:
                    detected_labels.add(label)  # 감지된 라벨 집합에 추가
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())  # 바운딩 박스 좌표
                    confidence = box.conf[0].item()  # 신뢰도
                    
                    # 바운딩 박스 및 텍스트 출력
                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(cv_image, f"{label} {confidence:.2f}", (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    self.get_logger().info(f"Detected: {label}, Confidence: {confidence:.2f}")
        
        # 감지된 객체가 있으면 UI에 전달, 없으면 "None" 전달
        msg = String()
        if detected_labels:
            detected_objects = ', '.join(detected_labels)
            msg.data = f"{detected_objects}"  # 감지된 객체 목록 전달
        else:
            msg.data = "None"  # 감지된 객체가 없을 경우 "None" 전달
        
        self.detected_object_publisher.publish(msg)
        
        # OpenCV 창을 업데이트
        cv2.imshow("YOLO Object Detector", cv_image)

def main(args=None):
    rclpy.init(args=args)
    node = YoloObjectDetector()

    # ROS2 콜백 루프 시작 (spin_once 사용)
    while rclpy.ok():
        rclpy.spin_once(node)  # 한 번의 콜백 처리
        cv2.waitKey(1)  # OpenCV 이벤트 처리

    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
