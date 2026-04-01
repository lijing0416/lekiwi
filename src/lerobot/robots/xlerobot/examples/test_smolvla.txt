import time
import math
import numpy as np
import cv2
import torch
import threading
from pathlib import Path
from transformers import AutoTokenizer
from lerobot.robots.so100_follower import SO100Follower, SO100FollowerConfig
from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
from lerobot.motors import Motor, MotorNormMode

# ================= 1. 动力学解算 (保持你的逻辑) =================

def body_to_wheel_raw(x, y, theta, wheel_radius=0.05, base_radius=0.125, max_raw=2800):
    theta_rad = theta * (np.pi / 180.0)
    velocity_vector = np.array([x, y, theta_rad])
    angles = np.radians(np.array([240, 0, 120]) - 90)
    m = np.array([[np.cos(a), np.sin(a), base_radius] for a in angles])
    wheel_speeds_ms = m.dot(velocity_vector)
    wheel_degps = (wheel_speeds_ms / wheel_radius) * (180.0 / np.pi)
    steps_per_deg = 4096.0 / 360.0
    raw = [int(round(d * steps_per_deg)) for d in wheel_degps]
    return {
        "base_left_wheel":  np.clip(raw[0], -max_raw, max_raw),
        "base_back_wheel":  np.clip(raw[1], -max_raw, max_raw),
        "base_right_wheel": np.clip(raw[2], -max_raw, max_raw),
    }

def inverse_kinematics(x, y, l1=0.1159, l2=0.1350):
    r = math.sqrt(x**2 + y**2)
    r = np.clip(r, 0.05, l1 + l2)
    theta1_offset = math.atan2(0.028, 0.11257)
    theta2_offset = math.atan2(0.0052, 0.1349) + theta1_offset
    cos_theta2 = -(r**2 - l1**2 - l2**2) / (2 * l1 * l2)
    theta2 = math.pi - math.acos(np.clip(cos_theta2, -1, 1))
    theta1 = math.atan2(y, x) + math.atan2(l2 * math.sin(theta2), l1 + l2 * math.cos(theta2))
    return 90 - math.degrees(theta1 + theta1_offset), math.degrees(theta2 + theta2_offset) - 90

# ================= 2. 异步推理大脑 =================

class VLABrain(threading.Thread):
    def __init__(self, model_path, instruction):
        super().__init__()
        self.device = "cpu" # 树莓派强制 CPU
        self.model_path = model_path
        self.instruction = instruction
        self.running = True
        self.latest_action = np.zeros(6) # [vx, vy, vt, dx, dy, gripper]
        
        print("正在初始化大脑 (SmolVLA)...")
        self.policy = SmolVLAPolicy.from_pretrained(model_path, local_files_only=True).to(self.device)
        self.tokenizer = AutoTokenizer.from_pretrained("HuggingFaceTB/SmolVLM2-500M-Video-Instruct")
        self.cap = cv2.VideoCapture(0) # 开启摄像头
        
    def run(self):
        while self.running:
            ret, frame = self.cap.read()
            if not ret: continue
            
            # 1. 预处理图像 (256x256)
            img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = cv2.resize(img, (256, 256))
            img_tensor = torch.from_numpy(img).permute(2, 0, 1).float().divide(255).unsqueeze(0).to(self.device)

            # 2. 构造输入
            inputs = self.tokenizer(self.instruction, return_tensors="pt").to(self.device)
            obs = {
                "observation.images.camera1": img_tensor,
                "observation.images.camera2": img_tensor, # 填充缺失视角
                "observation.images.camera3": img_tensor,
                "observation.state": torch.zeros(1, 6).to(self.device),
                "observation.language.tokens": inputs["input_ids"],
                "observation.language.attention_mask": inputs["attention_mask"].bool()
            }

            # 3. 推理 (耗时较长)
            with torch.inference_mode():
                action_chunk = self.policy.select_action(obs)
                self.latest_action = action_chunk[0].cpu().numpy() # 更新最新动作步
            
    def stop(self):
        self.running = False
        self.cap.release()

# ================= 3. 主执行循环 =================

def main():
    # 初始化机器人
    port = "/dev/ttyACM0"
    config = SO100FollowerConfig(port=port)
    robot = SO100Follower(config)
    
    # 注入底盘
    chassis_ids = {"base_left_wheel": 7, "base_back_wheel": 8, "base_right_wheel": 9}
    for name, mid in chassis_ids.items():
        robot.bus.motors[name] = Motor(mid, "sts3215", MotorNormMode.RANGE_M100_100)
        robot.bus._id_to_model_dict[mid] = "sts3215"

    # 启动大脑线程
    model_path = "/home/ljyyds/lekiwi/src/lerobot/model/smolvla_base"
    instruction = "pick up the gear"
    brain = VLABrain(model_path, instruction)
    brain.start()

    try:
        robot.connect(calibrate=False)
        for name in chassis_ids: robot.bus.write("Operating_Mode", name, 1) # 速度模式
        
        curr_x, curr_y = 0.1629, 0.1131
        targets = {k: 0.0 for k in ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]}
        
        print("\n[VLA 自动模式启动] 正在根据视觉指令控制 LeKiwi...")

        while True:
            # 获取大脑最新的“神经信号”
            act = brain.latest_action 
            
            # --- 底盘映射 ---
            vx = act[0] * 0.2  # 限制最大速度
            vy = act[1] * 0.2
            vt = act[2] * 30.0
            
            # --- 机械臂映射 (增量控制) ---
            curr_x += act[3] * 0.005 # 减小步幅，更平滑
            curr_y += act[4] * 0.005
            j2, j3 = inverse_kinematics(curr_x, curr_y)
            
            targets["shoulder_lift"], targets["elbow_flex"] = j2, j3
            targets["wrist_flex"] = -j2 - j3 # 保持夹爪水平
            targets["gripper"] = act[5] * 100 # 根据模型输出映射

            # 发送指令
            robot.bus.sync_write("Goal_Position", targets)
            robot.bus.sync_write("Goal_Velocity", body_to_wheel_raw(vx, vy, vt))
            
            time.sleep(0.05) # 20Hz 稳定输出

    except KeyboardInterrupt:
        print("\n用户停止...")
    finally:
        brain.stop()
        robot.bus.sync_write("Goal_Velocity", {n: 0 for n in chassis_ids})
        robot.disconnect()

if __name__ == "__main__":
    main()