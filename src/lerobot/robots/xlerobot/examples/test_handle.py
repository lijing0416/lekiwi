#!/usr/bin/env python3
import time
import math
import numpy as np
import traceback
import pygame
from lerobot.robots.so100_follower import SO100Follower, SO100FollowerConfig
from lerobot.motors import Motor, MotorNormMode

# ================= 1. 动力学逻辑 =================

def body_to_wheel_raw(x, y, theta, wheel_radius=0.05, base_radius=0.125, max_raw=2800):
    """底盘速度映射"""
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
    """机械臂 2D IK"""
    theta1_offset = math.atan2(0.028, 0.11257)
    theta2_offset = math.atan2(0.0052, 0.1349) + theta1_offset
    r = math.sqrt(x**2 + y**2)
    r = np.clip(r, 0.05, l1 + l2)
    cos_theta2 = -(r**2 - l1**2 - l2**2) / (2 * l1 * l2)
    theta2 = math.pi - math.acos(np.clip(cos_theta2, -1, 1))
    theta1 = math.atan2(y, x) + math.atan2(l2 * math.sin(theta2), l1 + l2 * math.cos(theta2))
    return 90 - math.degrees(theta1 + theta1_offset), math.degrees(theta2 + theta2_offset) - 90

# ================= 2. 手柄控制逻辑 =================

def control_loop(robot):
    # 初始化手柄
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("错误：未找到手柄，请连接手柄后重试！")
        return
    joy = pygame.joystick.Joystick(0)
    joy.init()
    print(f"手柄已连接: {joy.get_name()}")

    freq = 50
    dt = 1.0 / freq
    curr_x, curr_y = 0.1629, 0.1131
    pitch = 0.0
    targets = {k: 0.0 for k in ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]}
    
    print("\n[控制中] 左摇杆:底盘移动 | 右摇杆:机械臂X/Y | 十字键:云台/Pitch | RT:夹爪 | START:退出")

    while True:
        pygame.event.pump()
        
        # 退出判断 (通常 Start 键是 7)
        if joy.get_button(7): break

        # --- 底盘速度控制 (左摇杆) ---
        # Axis 0: 左右, Axis 1: 前后 (向上为负)
        vx = -joy.get_axis(1) * 0.3 if abs(joy.get_axis(1)) > 0.1 else 0.0
        vy = -joy.get_axis(0) * 0.3 if abs(joy.get_axis(0)) > 0.1 else 0.0
        
        # 自转控制 (LB/RB)
        vt = 0.0
        if joy.get_button(4): vt = 50.0  # LB 左转
        if joy.get_button(5): vt = -50.0 # RB 右转

        # --- 机械臂坐标控制 (右摇杆) ---
        # Axis 3: 左右, Axis 4: 前后
        if abs(joy.get_axis(4)) > 0.1: curr_x -= joy.get_axis(4) * 0.005
        if abs(joy.get_axis(3)) > 0.1: curr_y -= joy.get_axis(3) * 0.005
        
        j2, j3 = inverse_kinematics(curr_x, curr_y)
        targets["shoulder_lift"], targets["elbow_flex"] = j2, j3

        # --- 关节细节控制 ---
        # 十字键 (Hat) 控制 Pan 和 Pitch
        hat = joy.get_hat(0)
        targets["shoulder_pan"] -= hat[0] * 3
        pitch += hat[1] * 2

        # 腕部旋转 (X/B 键)
        if joy.get_button(2): targets["wrist_roll"] -= 5 # X
        if joy.get_button(1): targets["wrist_roll"] += 5 # B

        # 夹爪控制 (RT 触发器, Axis 5 或 2，视手柄而定)
        # 映射范围：从 -1.0(松) 到 1.0(紧) 映射为 0-100
        gripper_axis = joy.get_axis(5) 
        targets["gripper"] = (gripper_axis + 1.0) * 50

        # 计算腕部俯仰补偿
        targets["wrist_flex"] = -targets["shoulder_lift"] - targets["elbow_flex"] + pitch

        # --- 执行指令 ---
        try:
            robot.bus.sync_write("Goal_Position", targets)
            robot.bus.sync_write("Goal_Velocity", body_to_wheel_raw(vx, vy, vt))
            time.sleep(dt)
        except Exception:
            traceback.print_exc()
            break

# ================= 3. 主程序 =================

def main():
    port = input("串口 (默认 /dev/ttyACM0): ").strip() or "/dev/ttyACM0"
    config = SO100FollowerConfig(port=port)
    robot = SO100Follower(config)

    # 注入底盘电机 ID
    chassis_ids = {"base_left_wheel": 7, "base_back_wheel": 8, "base_right_wheel": 9}
    for name, mid in chassis_ids.items():
        robot.bus.motors[name] = Motor(mid, "sts3215", MotorNormMode.RANGE_M100_100)
        robot.bus._id_to_model_dict[mid] = "sts3215"

    try:
        robot.connect(calibrate=False)
        
        
        # 将底盘电机设为速度模式
        for name in chassis_ids:
            robot.bus.write("Operating_Mode", name, 1) 
        
        control_loop(robot)
    finally:
        print("\n正在停止机器人...")
        try:
            robot.bus.sync_write("Goal_Velocity", {n: 0 for n in chassis_ids})
            robot.disconnect()
        except: pass
        pygame.quit()

if __name__ == "__main__":
    main()
    