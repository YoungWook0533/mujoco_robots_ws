#!/usr/bin/env python3
import sys
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QLineEdit, QPushButton,
    QGridLayout
)
from PyQt5.QtCore import QTimer

class Ros2QtGui(Node, QWidget):
    def __init__(self):
        # 초기화
        rclpy.init(args=None)
        Node.__init__(self, 'ros2_qt_gui')
        QWidget.__init__(self)
        self.setWindowTitle('Husky FR3 Controller GUI')

        # Publishers & Subscribers
        self.mode_pub = self.create_publisher(Int32, 'husky_fr3_controller/mode_input', 10)
        self.pose_pub = self.create_publisher(PoseStamped, 'husky_fr3_controller/target_pose', 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.ee_sub = self.create_subscription(PoseStamped, 'husky_fr3_controller/ee_pose', self.ee_pose_callback, 10)

        # 현재 EE Pose 저장용
        self.current_ee = {'X':0.0,'Y':0.0,'Z':0.0,'Roll':0.0,'Pitch':0.0,'Yaw':0.0}

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
        grid.addWidget(QLabel('Mode 입력:'), row, 0)
        self.mode_input = QLineEdit()
        grid.addWidget(self.mode_input, row, 1)
        btn_mode = QPushButton('Publish Mode')
        btn_mode.clicked.connect(self._on_publish_mode)
        grid.addWidget(btn_mode, row, 2)
        row += 1

        # Target Pose 입력 (RPY)
        self.pose_inputs = {}
        for name in ['px','py','pz','roll','pitch','yaw']:
            grid.addWidget(QLabel(name + ':'), row, 0)
            le = QLineEdit()
            grid.addWidget(le, row, 1)
            self.pose_inputs[name] = le
            row += 1
        # Set Current / Publish buttons
        btn_set = QPushButton('Set Current')
        btn_set.clicked.connect(self._on_set_current)
        grid.addWidget(btn_set, row, 0)
        btn_pose = QPushButton('Publish Pose')
        btn_pose.clicked.connect(self._on_publish_pose)
        grid.addWidget(btn_pose, row, 1, 1, 2)
        row += 1

        # Joint States 표시 (세로)
        self.joint_edits = {}
        for i in range(7):
            jname = f'fr3_joint{i+1}'
            grid.addWidget(QLabel(jname + ':'), row, 0)
            le = QLineEdit(); le.setReadOnly(True)
            grid.addWidget(le, row, 1)
            self.joint_edits[jname] = le
            row += 1

        # EE Pose 표시 (세로)
        self.ee_edits = {}
        for label in ['X','Y','Z','Roll','Pitch','Yaw']:
            grid.addWidget(QLabel('EE ' + label + ':'), row, 0)
            le = QLineEdit(); le.setReadOnly(True)
            grid.addWidget(le, row, 1)
            self.ee_edits[label] = le
            row += 1

        self.setLayout(grid)
        self.resize(400, row * 30)

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
        msg = PoseStamped()
        try:
            msg.header.frame_id = "world"
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.x = float(self.pose_inputs['px'].text())
            msg.pose.position.y = float(self.pose_inputs['py'].text())
            msg.pose.position.z = float(self.pose_inputs['pz'].text())
            # RPY → quaternion
            roll = math.radians(float(self.pose_inputs['roll'].text()))
            pitch = math.radians(float(self.pose_inputs['pitch'].text()))
            yaw = math.radians(float(self.pose_inputs['yaw'].text()))
            cy = math.cos(yaw*0.5); sy = math.sin(yaw*0.5)
            cp = math.cos(pitch*0.5); sp = math.sin(pitch*0.5)
            cr = math.cos(roll*0.5); sr = math.sin(roll*0.5)
            msg.pose.orientation.w = cr*cp*cy + sr*sp*sy
            msg.pose.orientation.x = sr*cp*cy - cr*sp*sy
            msg.pose.orientation.y = cr*sp*cy + sr*cp*sy
            msg.pose.orientation.z = cr*cp*sy - sr*sp*cy
        except Exception:
            msg = PoseStamped()
        self.pose_pub.publish(msg)

    def _on_set_current(self):
        # 현재 EE 값을 Target Pose 입력 필드에 채움
        mapping = {'px':'X','py':'Y','pz':'Z','roll':'Roll','pitch':'Pitch','yaw':'Yaw'}
        for input_key, ee_key in mapping.items():
            val = self.current_ee.get(ee_key, 0.0)
            # roll/pitch/yaw 입력은 degree
            if input_key in ['roll','pitch','yaw']:
                val = math.degrees(val)
            self.pose_inputs[input_key].setText(f"{val:.3f}")

    def joint_state_callback(self, msg: JointState):
        for i, pos in enumerate(msg.position[:7]):
            self.joint_edits[f'fr3_joint{i+1}'].setText(f"{pos:.3f}")

    def ee_pose_callback(self, msg: PoseStamped):
        # quaternion -> RPY
        x,y,z,w = msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w
        sinr = 2*(w*x + y*z); cosr = 1-2*(x*x + y*y)
        roll = math.atan2(sinr, cosr)
        sinp = 2*(w*y - z*x)
        pitch = math.copysign(math.pi/2, sinp) if abs(sinp)>=1 else math.asin(sinp)
        siny = 2*(w*z + x*y); cosy = 1-2*(y*y + z*z)
        yaw = math.atan2(siny, cosy)
        # 저장 및 표시
        vals = {'X':msg.pose.position.x, 'Y':msg.pose.position.y, 'Z':msg.pose.position.z,
                'Roll':roll, 'Pitch':pitch, 'Yaw':yaw}
        for key, val in vals.items():
            self.current_ee[key] = val
            self.ee_edits[key].setText(f"{math.degrees(val) if key in ['Roll','Pitch','Yaw'] else val:.3f}")

    def closeEvent(self, event):
        self.timer.stop()
        rclpy.shutdown()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = Ros2QtGui()
    try:
        gui.show()
    except KeyboardInterrupt:
        pass
    finally:
        sys.exit(app.exec_())