import sys
import random
import threading
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout, QTextEdit
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt, QMetaObject, QTimer , pyqtSignal
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import SetInitialPose
from rclpy.action import ActionClient
from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import String  # YOLO 객체 인식 결과를 처리할 메시지 타입

class SmartFarmUI(QWidget):

    detection_signal = pyqtSignal(str)
    def __init__(self):
        super().__init__()
        self.initUI()
        self.detection_signal.connect(self.update_detection_status)
        self.last_detected_class = "없음"

    def initUI(self):
        self.setWindowTitle("🌱 스마트팜 로봇 UI")
        self.setGeometry(100, 100, 800, 600)  # 윈도우 크기 조정
        self.setStyleSheet("background-color: #e0f7fa;")

        title_label = QLabel("🚜 스마트팜 로봇 제어 시스템", self)
        title_label.setFont(QFont("Arial", 16, QFont.Bold))
        title_label.setAlignment(Qt.AlignCenter)

        # 상태 표시 (로봇 2대)
        self.status_label_1 = QLabel("로봇1 상태: 대기 중", self)
        self.status_label_2 = QLabel("로봇2 상태: 대기 중", self)

        # 탐지 현황 레이아웃
        self.detection_status_label = QLabel("과일 탐지 현황:", self)
        self.detection_label = QLabel("과일: 없음", self)
        self.detection_status = QLabel("상태: 탐지되지 않음", self)

        # 탐지 현황 레이아웃 스타일
        self.detection_label.setStyleSheet("color: #00796b; background-color: #b2dfdb; padding: 5px; border-radius: 5px;")
        self.detection_status.setStyleSheet("color: #f44336; background-color: #ffebee; padding: 5px; border-radius: 5px;")

        # 로그 창
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setStyleSheet("background-color: #ffffff; border: 1px solid #b0bec5; padding: 5px;")

        # 기본 버튼 스타일

        # "탐지 상태 갱신" 버튼 추가
        self.refresh_button = QPushButton("탐지 상태 갱신")
        self.refresh_button.setStyleSheet("""
            QPushButton {
                background-color: #FFC107
;
                color: black;
                font-size: 14px;
                padding: 10px;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #FFA000
;
            }
        """)
        self.refresh_button.clicked.connect(self.manual_refresh)

        button_style = """
            QPushButton {
                background-color: #4caf50;
                color: white;
                font-size: 14px;
                padding: 10px;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #388e3c;
            }
        """

        # 탐색 및 복귀 버튼 스타일
        search_button_style = """
            QPushButton {
                background-color: #2196f3;  # 파란색
                color: white;
                font-size: 14px;
                padding: 10px;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #1976d2;
            }
        """

        return_button_style = """
            QPushButton {
                background-color: #ff9800;  # 주황색
                color: white;
                font-size: 14px;
                padding: 10px;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #f57c00;
            }
        """

        # 버튼 생성 함수
        def create_button(text, robot, style):
            button = QPushButton(text)
            button.setStyleSheet(style)
            button.clicked.connect(lambda: self.log_action(robot, text))
            return button

        # 로봇 1 버튼
        self.water_button_1 = create_button("💧 로봇1 - 물 주기", "로봇1", button_style)
        self.fertilize_button_1 = create_button("🌿 로봇1 - 비료 주기", "로봇1", button_style)
        self.remove_weeds_button_1 = create_button("🪓 로봇1 - 잡초 제거", "로봇1", button_style)
        self.prune_button_1 = create_button("✂️ 로봇1 - 가지 관리", "로봇1", button_style)
        self.remove_sick_button_1 = create_button("⚠️ 로봇1 - 병든 작물 제거", "로봇1", button_style)
        self.harvest_button_1 = create_button("🌾 로봇1 - 수확", "로봇1", button_style)

        # 로봇 2 버튼
        self.water_button_2 = create_button("💧 로봇2 - 물 주기", "로봇2", button_style)
        self.fertilize_button_2 = create_button("🌿 로봇2 - 비료 주기", "로봇2", button_style)
        self.remove_weeds_button_2 = create_button("🪓 로봇2 - 잡초 제거", "로봇2", button_style)
        self.prune_button_2 = create_button("✂️ 로봇2 - 가지 관리", "로봇2", button_style)
        self.remove_sick_button_2 = create_button("⚠️ 로봇2 - 병든 작물 제거", "로봇2", button_style)
        self.harvest_button_2 = create_button("🌾 로봇2 - 수확", "로봇2", button_style)

        # 탐색 버튼 (8개의 지정된 지점 중 하나로 이동)
        self.search_button_1 = create_button("🔍 로봇1 - 탐색 시작", "로봇1", search_button_style)
        self.search_button_2 = create_button("🔍 로봇2 - 탐색 시작", "로봇2", search_button_style)

        # 복귀 버튼 (시작 위치로 돌아가기)
        self.return_button_1 = create_button("↩️ 로봇1 - 복귀", "로봇1", return_button_style)
        self.return_button_2 = create_button("↩️ 로봇2 - 복귀", "로봇2", return_button_style)

        # 레이아웃 설정
        main_layout = QVBoxLayout()
        main_layout.addWidget(title_label)
        main_layout.addWidget(self.status_label_1)
        main_layout.addWidget(self.refresh_button)
        button_layout_1 = QHBoxLayout()
        button_layout_1.addWidget(self.water_button_1)
        button_layout_1.addWidget(self.fertilize_button_1)
        button_layout_1.addWidget(self.remove_weeds_button_1)
        button_layout_1.addWidget(self.prune_button_1)
        button_layout_1.addWidget(self.remove_sick_button_1)
        button_layout_1.addWidget(self.harvest_button_1)

        main_layout.addLayout(button_layout_1)
        main_layout.addWidget(self.status_label_2)

        button_layout_2 = QHBoxLayout()
        button_layout_2.addWidget(self.water_button_2)
        button_layout_2.addWidget(self.fertilize_button_2)
        button_layout_2.addWidget(self.remove_weeds_button_2)
        button_layout_2.addWidget(self.prune_button_2)
        button_layout_2.addWidget(self.remove_sick_button_2)
        button_layout_2.addWidget(self.harvest_button_2)

        main_layout.addLayout(button_layout_2)

        # 탐지 현황 레이아웃 추가
        main_layout.addWidget(self.detection_status_label)
        main_layout.addWidget(self.detection_label)
        main_layout.addWidget(self.detection_status)

        # 새로운 버튼을 오른쪽에 배치하는 레이아웃
        button_layout_3 = QVBoxLayout()
        button_layout_3.addWidget(self.search_button_1)
        button_layout_3.addWidget(self.return_button_1)
        button_layout_3.addWidget(self.search_button_2)
        button_layout_3.addWidget(self.return_button_2)

        main_layout.addLayout(button_layout_3)
        main_layout.addWidget(QLabel("📝 작업 로그"))
        main_layout.addWidget(self.log_text)

        self.setLayout(main_layout)

        # ROS2 관련 초기화
        rclpy.init()
        self.node = SmartFarmNode(self)
        self.node.set_initial_pose()

        # 버튼 동작에 연결
        self.search_button_1.clicked.connect(lambda: self.search_action("로봇1"))
        self.search_button_2.clicked.connect(lambda: self.search_action("로봇2"))
        self.return_button_1.clicked.connect(lambda: self.return_action("로봇1"))
        self.return_button_2.clicked.connect(lambda: self.return_action("로봇2"))

    def log_action(self, robot, action):
        log_message = f"{robot}: {action} 실행"
        self.log_text.append(log_message)
        if robot == "로봇1":
            self.status_label_1.setText(f"{robot} 상태: {action}")
        else:
            self.status_label_2.setText(f"{robot} 상태: {action}")

    def search_action(self, robot):
        log_message = f"{robot}: 탐색 시작"
        self.log_text.append(log_message)
        if robot == "로봇1":
            # 로봇1이 랜덤한 위치로 이동
            self.node.navigate_to_pose(self.node.get_random_point())
        else:
            # 로봇2가 랜덤한 위치로 이동
            self.node.navigate_to_pose(self.node.get_random_point())

    def return_action(self, robot):
        log_message = f"{robot}: 복귀 시작"
        self.log_text.append(log_message)
        if robot == "로봇1":
            self.node.return_to_initial_position()
        else:
            self.node.return_to_initial_position()

    def update_detection_status(self, detected_class):
        """과일 탐지 상태를 UI에 반영"""
        self.last_detected_class = detected_class  # 최신 탐지 결과 저장
        self.detection_label.setText(f"과일: {detected_class}")
        self.detection_status.setText("상태: 탐지됨" if detected_class in ['apple', 'orange', 'banana'] else "상태: 탐지되지 않음")    
        
    def manual_refresh(self):

        self.update_detection_status(self.last_detected_class)

class SmartFarmNode(Node):
    def __init__(self, ui):
        super().__init__('smart_farm_node')
        self.ui = ui

        # 8개의 지정된 지점
        self.predefined_points = [
            [-2.3, -10,0], [-7.5, -2], [-13.0, -6.0], 
            [-6.0,10.3 ], [-0.36, 13.7], [4.0, 5.15], 
            [9.5, -2.8], [4.2, -6.12]
        ]
        self.initial_pose = [0.0, 0.0, 0.0, 1.0]  # 초기 로봇 위치 (x, y, z, w)

        # 액션 클라이언트 생성
        self.navigate_to_pose_action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # 서비스 클라이언트 생성
        self.set_initial_pose_service_client = self.create_client(SetInitialPose, '/set_initial_pose')

        # YOLO 인식 결과를 받기 위한 Subscriber 설정
        self.create_subscription(String, '/detected_object', self.yolo_callback, 10)

    def set_initial_pose(self):
        request = SetInitialPose.Request()
        request.pose.header.frame_id = 'map'
        request.pose.pose.pose.position = Point(x=self.initial_pose[0], y=self.initial_pose[1], z=0.0)
        request.pose.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=self.initial_pose[2], w=self.initial_pose[3])
        future = self.set_initial_pose_service_client.call_async(request)
        future.add_done_callback(self.initial_pose_callback)

    def initial_pose_callback(self, future):
        if future.result():
            print("[INFO] 초기 위치 설정 성공")
        else:
            print("[ERROR] 초기 위치 설정 실패")

    def navigate_to_pose(self, point):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = point[0]
        goal_msg.pose.pose.position.y = point[1]
        goal_msg.pose.pose.orientation.w = 1.0

        self.send_goal_future = self.navigate_to_pose_action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.navigate_to_pose_callback)

    def navigate_to_pose_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            print("[ERROR] 이동 목표가 거부됨")
        else:
            print("[INFO] 로봇이 이동을 시작했습니다.")

    def return_to_initial_position(self):
        self.navigate_to_pose(self.initial_pose[:2])

    def get_random_point(self):
        return random.choice(self.predefined_points)

    def yolo_callback(self, msg):
        
        detected_class = msg.data  # YOLO 결과로 받은 과일 클래스

        print(msg.data)
        self.ui.last_detected_class=detected_class

def ros_thread_func(node):
    rclpy.spin(node)
    rclpy.shutdown()

def main():
    # Qt 어플리케이션과 ROS2 노드 실행을 위한 스레드 처리
    app = QApplication(sys.argv)
    window = SmartFarmUI()

    ros_thread = threading.Thread(target=ros_thread_func, args=(window.node,))
    ros_thread.start()

    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
