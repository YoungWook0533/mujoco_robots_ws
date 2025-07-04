#!/usr/bin/env python3
import sys
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import JointState

from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QLineEdit, QPushButton,
    QGridLayout
)
from PyQt5.QtCore import QTimer

class XLSQtGui(Node, QWidget):
    def __init__(self):
        # ROS2 & Qt 초기화
        rclpy.init(args=None)
        Node.__init__(self, 'xls_controller_gui')
        QWidget.__init__(self)
        self.setWindowTitle('XLS Controller GUI')

        # Publishers
        self.mode_pub = self.create_publisher(Int32, 'xls_controller/mode_input', 10)
        self.pose_pub = self.create_publisher(Pose, 'xls_controller/target_pose', 10)
        self.vel_pub  = self.create_publisher(Twist, 'xls_controller/cmd_vel', 10)

        # Subscribers
        self.joint_sub    = self.create_subscription(JointState, 'xls_controller/joint_states', self.joint_state_callback, 10)
        self.base_pose_sub = self.create_subscription(Pose, 'xls_controller/base_pose', self.base_pose_callback, 10)
        self.base_vel_sub  = self.create_subscription(Twist, 'xls_controller/base_vel', self.base_vel_callback, 10)

        # 내부 상태 저장
        self.current_base_pose = {'x':0.0, 'y':0.0, 'theta':0.0}
        self.current_base_vel  = {'vx':0.0,'vy':0.0,'w':0.0}

        # UI 구성
        self._build_ui()

        # Qt 타이머로 ROS spin_once 호출
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._spin_once)
        self.timer.start(50)

    def _build_ui(self):
        grid = QGridLayout()
        row = 0

        # Mode 입력
        grid.addWidget(QLabel('Mode (1: stop,2: vel,3: pose):'), row, 0)
        self.mode_input = QLineEdit()
        grid.addWidget(self.mode_input, row, 1)
        btn_mode = QPushButton('Publish Mode')
        btn_mode.clicked.connect(self._on_publish_mode)
        grid.addWidget(btn_mode, row, 2)
        row += 1

        # Target Pose 입력 (x, y, theta)
        self.pose_inputs = {}
        for name in ['x','y','theta']:
            grid.addWidget(QLabel(f'Target {name}:'), row, 0)
            le = QLineEdit()
            grid.addWidget(le, row, 1)
            self.pose_inputs[name] = le
            row += 1
        btn_pose = QPushButton('Publish Pose')
        btn_pose.clicked.connect(self._on_publish_pose)
        grid.addWidget(btn_pose, row, 0, 1, 3)
        row += 1

        # Target Velocity 입력 (vx, vy, w)
        self.vel_inputs = {}
        for name in ['vx', 'vy','w']:
            grid.addWidget(QLabel(f'Target {name}:'), row, 0)
            le = QLineEdit()
            grid.addWidget(le, row, 1)
            self.vel_inputs[name] = le
            row += 1
        btn_vel = QPushButton('Publish Velocity')
        btn_vel.clicked.connect(self._on_publish_vel)
        grid.addWidget(btn_vel, row, 0, 1, 3)
        row += 1

        # Joint States 표시 (4 wheels)
        self.joint_edits = {}
        for jname in ['front_left_wheel', 'front_right_wheel', 'rear_left_wheel', 'rear_right_wheel']:
            grid.addWidget(QLabel(jname+':' ), row, 0)
            le = QLineEdit(); le.setReadOnly(True)
            grid.addWidget(le, row, 1)
            self.joint_edits[jname] = le
            row += 1

        # Base Pose 표시
        self.base_pose_edits = {}
        for label in ['x','y','theta']:
            grid.addWidget(QLabel('Base '+label+':'), row, 0)
            le = QLineEdit(); le.setReadOnly(True)
            grid.addWidget(le, row, 1)
            self.base_pose_edits[label] = le
            row += 1

        # Base Velocity 표시
        self.base_vel_edits = {}
        for label in ['vx','vy','w']:
            grid.addWidget(QLabel('Vel '+label+':'), row, 0)
            le = QLineEdit(); le.setReadOnly(True)
            grid.addWidget(le, row, 1)
            self.base_vel_edits[label] = le
            row += 1

        self.setLayout(grid)
        self.resize(400, row*30)

    def _spin_once(self):
        try:
            rclpy.spin_once(self, timeout_sec=0)
        except Exception:
            pass

    def _on_publish_mode(self):
        msg = Int32()
        try: msg.data = int(self.mode_input.text())
        except: msg.data = 0
        self.mode_pub.publish(msg)

    def _on_publish_pose(self):
        msg = Pose()
        try:
            msg.position.x = float(self.pose_inputs['x'].text())
            msg.position.y = float(self.pose_inputs['y'].text())
            theta = math.radians(float(self.pose_inputs['theta'].text()))
            # Only yaw
            cy = math.cos(theta*0.5); sy = math.sin(theta*0.5)
            msg.orientation.w = cy
            msg.orientation.x = 0.0
            msg.orientation.y = 0.0
            msg.orientation.z = sy
        except Exception:
            msg = Pose()
        self.pose_pub.publish(msg)

    def _on_publish_vel(self):
        msg = Twist()
        try:
            msg.linear.x = float(self.vel_inputs['vx'].text())
            msg.linear.y = float(self.vel_inputs['vy'].text())
            msg.angular.z = float(self.vel_inputs['w'].text())
        except:
            msg = Twist()
        self.vel_pub.publish(msg)

    def joint_state_callback(self, msg: JointState):
        for name, pos in zip(msg.name, msg.velocity):
            if name in self.joint_edits:
                self.joint_edits[name].setText(f"{pos:.3f}")

    def base_pose_callback(self, msg: Pose):
        x = msg.position.x; y = msg.position.y
        # orientation -> yaw
        siny = 2*(msg.orientation.w*msg.orientation.z + msg.orientation.x*msg.orientation.y)
        cosy = 1-2*(msg.orientation.y**2 + msg.orientation.z**2)
        theta = math.atan2(siny, cosy)
        self.current_base_pose.update({'x':x,'y':y,'theta':theta})
        for label in ['x','y','theta']:
            val = self.current_base_pose[label]
            if label=='theta': val = math.degrees(val)
            self.base_pose_edits[label].setText(f"{val:.3f}")

    def base_vel_callback(self, msg: Twist):
        vx = msg.linear.x; vy = msg.linear.y; w = msg.angular.z
        self.current_base_vel.update({'vx':vx,'vy':vy,'w':w})
        for label in ['vx','vy','w']:
            val = self.current_base_vel[label]
            self.base_vel_edits[label].setText(f"{val:.3f}")

    def closeEvent(self, event):
        self.timer.stop()
        rclpy.shutdown()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = XLSQtGui()
    gui.show()
    sys.exit(app.exec_())