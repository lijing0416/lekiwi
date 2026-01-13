#!/usr/bin/env python3
import time
import math
import numpy as np
import traceback
import copy
from lerobot.robots.so100_follower import SO100Follower, SO100FollowerConfig
from lerobot.teleoperators.keyboard import KeyboardTeleop, KeyboardTeleopConfig
from lerobot.motors import Motor, MotorNormMode

# ================= 1. 动力学逻辑 =================

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
    theta1_offset = math.atan2(0.028, 0.11257)
    theta2_offset = math.atan2(0.0052, 0.1349) + theta1_offset
    r = math.sqrt(x**2 + y**2)
    r = np.clip(r, 0.05, l1 + l2)
    cos_theta2 = -(r**2 - l1**2 - l2**2) / (2 * l1 * l2)
    theta2 = math.pi - math.acos(np.clip(cos_theta2, -1, 1))
    theta1 = math.atan2(y, x) + math.atan2(l2 * math.sin(theta2), l1 + l2 * math.cos(theta2))
    return 90 - math.degrees(theta1 + theta1_offset), math.degrees(theta2 + theta2_offset) - 90

# ================= 2. 主控制逻辑 =================

def control_loop(robot, keyboard):
    freq = 50
    dt = 1.0 / freq
    curr_x, curr_y = 0.1629, 0.1131
    pitch = 0.0
    targets = {k: 0.0 for k in ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]}
    
    print("\n[控制中] 底盘: IJKL/UO | 机械臂: WSED/QA/TG/YH/RF | 退出: X")

    while True:
        try:
            act = keyboard.get_action()
            vx, vy, vt = 0.0, 0.0, 0.0

            if act:
                if "x" in act: break
                # 底盘速度
                if "i" in act: vx = 0.25
                if "k" in act: vx = -0.25
                if "j" in act: vy = 0.25
                if "l" in act: vy = -0.25
                if "u" in act: vt = 40.0
                if "o" in act: vt = -40.0
                # 机械臂 IK
                if "w" in act: curr_x -= 0.005
                if "s" in act: curr_x += 0.005
                if "e" in act: curr_y -= 0.005
                if "d" in act: curr_y += 0.005
                j2, j3 = inverse_kinematics(curr_x, curr_y)
                targets["shoulder_lift"], targets["elbow_flex"] = j2, j3
                # 关节
                if "q" in act: targets["shoulder_pan"] -= 3
                if "a" in act: targets["shoulder_pan"] += 3
                if "t" in act: targets["wrist_roll"] -= 8
                if "g" in act: targets["wrist_roll"] += 8
                if "y" in act: targets["gripper"] -= 15
                if "h" in act: targets["gripper"] += 15
                if "r" in act: pitch += 3
                if "f" in act: pitch -= 3

            targets["wrist_flex"] = -targets["shoulder_lift"] - targets["elbow_flex"] + pitch

            # 发送指令
            robot.bus.sync_write("Goal_Position", targets)
            robot.bus.sync_write("Goal_Velocity", body_to_wheel_raw(vx, vy, vt))
            time.sleep(dt)
        except:
            traceback.print_exc()
            break

# ================= 3. 程序入口 =================

def main():
    port = input("串口 (默认 /dev/ttyACM0): ").strip() or "/dev/ttyACM0"
    config = SO100FollowerConfig(port=port)
    robot = SO100Follower(config)

    # --- 核心修复：双向注入映射 ---
    print("注入底盘电机 ID 7, 8, 9...")
    chassis_ids = {"base_left_wheel": 7, "base_back_wheel": 8, "base_right_wheel": 9}
    
    for name, mid in chassis_ids.items():
        # 1. 注入电机对象
        new_motor = Motor(mid, "sts3215", MotorNormMode.RANGE_M100_100)
        robot.bus.motors[name] = new_motor
        # 2. 注入 ID 到型号的查找字典 (解决 KeyError: 7)
        robot.bus._id_to_model_dict[mid] = "sts3215"

    keyboard = KeyboardTeleop(KeyboardTeleopConfig())

    try:
        # 必须 calibrate=False，因为轮子无法进行位置限制读取
        robot.connect(calibrate=False)
        keyboard.connect()

        print("配置底盘为速度模式...")
        for name in chassis_ids:
            robot.bus.write("Operating_Mode", name, 1) # 1 = Velocity
        
        control_loop(robot, keyboard)
    finally:
        print("\n停止...")
        try:
            robot.bus.sync_write("Goal_Velocity", {n: 0 for n in chassis_ids})
            robot.disconnect()
        except: pass

if __name__ == "__main__":
    main()