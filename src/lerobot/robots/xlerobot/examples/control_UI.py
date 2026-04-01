import sys
import cv2
import math
import numpy as np
from PySide6.QtWidgets import (QApplication, QMainWindow, QSizePolicy, QWidget, QVBoxLayout, 
                             QHBoxLayout, QSlider, QLabel, QPushButton, QDoubleSpinBox, QGroupBox, QGridLayout)
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QImage, QPixmap

# 尝试导入硬件库
try:
    from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig
    HAS_HARDWARE = True
except ImportError: 
    HAS_HARDWARE = False
    print("警告: 未检测到 lerobot 库，将以演示模式启动。")

# ================= 1. 核心算法 (仅保留手臂逆解) =================

def inverse_kinematics(x, y, l1=0.1159, l2=0.1350):
    theta1_offset = math.atan2(0.028, 0.11257)
    theta2_offset = math.atan2(0.0052, 0.1349) + theta1_offset
    r = math.sqrt(x**2 + y**2)
    r = np.clip(r, 0.05, l1 + l2)
    cos_theta2 = -(r**2 - l1**2 - l2**2) / (2 * l1 * l2)
    theta2 = math.pi - math.acos(np.clip(cos_theta2, -1, 1))
    theta1 = math.atan2(y, x) + math.atan2(l2 * math.sin(theta2), l1 + l2 * math.cos(theta2))
    return 90 - math.degrees(theta1 + theta1_offset), math.degrees(theta2 + theta2_offset) - 90

# ================= 2. UI 界面类 =================

class SO101Controller(QMainWindow):
    def __init__(self, robot=None):
        super().__init__()
        self.robot = robot
        self.setWindowTitle("SO101 机械臂控制台 (精简版)")
        self.resize(1100, 800)

        # 核心状态变量
        self.curr_x, self.curr_y = 0.1629, 0.1131
        self.pitch = 0.0
        # 只保留手臂 6 个关节
        self.targets = {k: 0.0 for k in ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]}

        self._setup_ui()
        
        # 视频刷新 (30fps)
        self.cap = cv2.VideoCapture(2)
        self.video_timer = QTimer()
        self.video_timer.timeout.connect(self._update_frame)
        self.video_timer.start(33)

        # 硬件指令发送 (20Hz)
        self.cmd_timer = QTimer()
        self.cmd_timer.timeout.connect(self._send_commands)
        self.cmd_timer.start(50)

    def _setup_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QHBoxLayout(main_widget)

        # --- 左侧：实时监控 ---
        monitor_layout = QVBoxLayout()
        self.video_label = QLabel("正在连接相机...")
        self.video_label.setStyleSheet("background: black; border: 2px solid #333;")
        self.video_label.setAlignment(Qt.AlignCenter)
        # 核心修复：防止内容撑大布局
        self.video_label.setMinimumSize(640, 480)
        self.video_label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored) 
        monitor_layout.addWidget(self.video_label, 5)

        self.status_bar = QLabel("状态: 准备就绪")
        self.status_bar.setStyleSheet("font-size: 14px; color: green; padding: 5px;")
        monitor_layout.addWidget(self.status_bar)
        layout.addLayout(monitor_layout, 2)

        # --- 右侧：控制面板 ---
        ctrl_layout = QVBoxLayout()

        # 急停
        stop_btn = QPushButton("STOP / 停止指令")
        stop_btn.setStyleSheet("background: red; color: white; font-weight: bold; height: 50px;")
        stop_btn.clicked.connect(self.emergency_stop)
        ctrl_layout.addWidget(stop_btn)

        # XYZ 逆解控制
        ik_group = QGroupBox("末端逆解控制 (IK)")
        ik_grid = QGridLayout()
        self.spin_x = self._create_spin("X (前后)", 0.1629, ik_grid, 0)
        self.spin_y = self._create_spin("Y (上下)", 0.1131, ik_grid, 1)
        self.spin_p = self._create_spin("Pitch", 0.0, ik_grid, 2, is_deg=True)
        ik_group.setLayout(ik_grid)
        ctrl_layout.addWidget(ik_group)

        # 关节滑条
        joint_group = QGroupBox("手动关节控制")
        j_layout = QVBoxLayout()
        for j_name in ["shoulder_pan", "wrist_roll", "gripper"]:
            j_layout.addWidget(QLabel(f"{j_name}:"))
            sld = QSlider(Qt.Horizontal)
            sld.setRange(-150, 150)
            sld.valueChanged.connect(lambda v, n=j_name: self.update_target(n, v))
            j_layout.addWidget(sld)
        joint_group.setLayout(j_layout)
        ctrl_layout.addWidget(joint_group)

        layout.addLayout(ctrl_layout, 1)

    def _create_spin(self, name, val, grid, row, is_deg=False):
        grid.addWidget(QLabel(name), row, 0)
        spin = QDoubleSpinBox()
        spin.setRange(-1.0, 1.0) if not is_deg else spin.setRange(-150, 150)
        spin.setSingleStep(0.005)
        spin.setValue(val)
        spin.valueChanged.connect(self.sync_ik)
        grid.addWidget(spin, row, 1)
        return spin

    def sync_ik(self):
        self.curr_x = self.spin_x.value()
        self.curr_y = self.spin_y.value()
        self.pitch = self.spin_p.value()
        j2, j3 = inverse_kinematics(self.curr_x, self.curr_y)
        self.targets["shoulder_lift"], self.targets["elbow_flex"] = j2, j3

    def update_target(self, name, val):
        self.targets[name] = float(val)

    def _update_frame(self):
        ret, frame = self.cap.read()
        if ret:
            frame = cv2.flip(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB), 1)
            h, w, ch = frame.shape
            img = QImage(frame.data, w, h, ch*w, QImage.Format_RGB888)
            
            # 核心修复：使用平滑缩放，并固定参照
            pixmap = QPixmap.fromImage(img)
            scaled_pixmap = pixmap.scaled(
                self.video_label.width(), 
                self.video_label.height(), 
                Qt.KeepAspectRatio, 
                Qt.SmoothTransformation
            )
            self.video_label.setPixmap(scaled_pixmap)

    def _send_commands(self):
        if not self.robot: return
        
        # 计算腕部补偿
        self.targets["wrist_flex"] = -self.targets["shoulder_lift"] - self.targets["elbow_flex"] + self.pitch
        
        try:
            # 既然已校准，直接使用 sync_write 发送手臂 6 轴
            self.robot.bus.sync_write("Goal_Position", self.targets)
            self.status_bar.setText("状态: 通信正常")
            self.status_bar.setStyleSheet("color: green;")
        except Exception as err:
            self.status_bar.setText(f"状态: 通信异常 ({type(err).__name__})")
            self.status_bar.setStyleSheet("color: red;")

    def emergency_stop(self):
        if self.cmd_timer.isActive():
            self.cmd_timer.stop()
            self.status_bar.setText("状态: 指令已停止发送")
        else:
            self.cmd_timer.start(50)
            self.status_bar.setText("状态: 恢复发送")

# ================= 3. 启动逻辑 =================

if __name__ == "__main__":
    app = QApplication(sys.argv)
    robot = None
    
    if HAS_HARDWARE:
        try:
            port = "/dev/ttyACM0" 
            config = SO101FollowerConfig(port=port)
            robot = SO101Follower(config)
            
            # 不再手动注入 7, 8, 9 电机
            # 直接连接，它会自动寻找并加载你本地的 calibration 文件夹
            robot.connect() 
            print("硬件连接成功并加载校准数据。")
        except Exception as e:
            print(f"硬件连接失败: {e}")
            robot = None

    gui = SO101Controller(robot)
    gui.show()
    sys.exit(app.exec())