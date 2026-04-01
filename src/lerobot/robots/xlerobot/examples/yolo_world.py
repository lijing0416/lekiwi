#!/usr/bin/env python3
import time
import math
import cv2
import numpy as np
import traceback
from ultralytics import YOLOWorld

# --- 硬件与运动学配置 ---
JOINT_CALIBRATION = [
    ['shoulder_pan', 6.0, 1.0],
    ['shoulder_lift', 2.0, 0.97],
    ['elbow_flex', 0.0, 1.05],
    ['wrist_flex', 0.0, 0.94],
    ['wrist_roll', 0.0, 0.5],
    ['gripper', 0.0, 1.0],
]

K_pan = -0.006  
K_y = 0.00004   

def apply_joint_calibration(joint_name, raw_position):
    for joint_cal in JOINT_CALIBRATION:
        if joint_cal[0] == joint_name:
            return (raw_position - joint_cal[1]) * joint_cal[2]
    return raw_position

def inverse_kinematics(x, y, l1=0.1159, l2=0.1350):
    theta1_offset = math.atan2(0.028, 0.11257)
    theta2_offset = math.atan2(0.0052, 0.1349) + theta1_offset
    r = math.sqrt(x**2 + y**2)
    r = max(abs(l1-l2), min(l1+l2, r))
    cos_theta2 = -(r**2 - l1**2 - l2**2) / (2 * l1 * l2)
    theta2 = math.pi - math.acos(cos_theta2)
    beta = math.atan2(y, x)
    gamma = math.atan2(l2 * math.sin(theta2), l1 + l2 * math.cos(theta2))
    theta1 = beta + gamma
    joint2_deg = 90 - math.degrees(theta1 + theta1_offset)
    joint3_deg = math.degrees(theta2 + theta2_offset) - 90
    return joint2_deg, joint3_deg

# --- 核心视觉逻辑 ---
def vision_control_update(target_positions, current_x, current_y, model, cap, target_objects):
    ret, frame = cap.read()
    if not ret: return current_x, current_y

    results = model.predict(frame, conf=0.2, verbose=False)
    annotated_frame = frame.copy()

    if results and results[0].boxes:
        for box in results[0].boxes:
            # 1. 位置计算
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            h, w = frame.shape[:2]
            dx, dy = cx - w // 2, cy - h // 2

            # 2. 旋转角度提取 (OpenCV MinAreaRect)
            roi = frame[max(0, y1):y2, max(0, x1):x2]
            angle = 0
            if roi.size > 0:
                gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
                _, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
                contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if contours:
                    cnt = max(contours, key=cv2.contourArea)
                    rect = cv2.minAreaRect(cnt)
                    angle = rect[2]
                    if rect[1][0] < rect[1][1]: angle -= 90
            
            # 3. 映射到控制量
            # A. 左右对准
            d_pan = -K_pan * dx
            if abs(d_pan) > 0.05: target_positions['shoulder_pan'] += d_pan
            
            # B. 远近对准 (IK)
            d_cy = -K_y * dy
            current_y += max(min(d_cy, 0.005), -0.005) if abs(d_cy) > 0.0005 else 0
            
            # C. 旋转对准 (对齐长轴)
            target_positions['wrist_roll'] = angle 

            # D. 自动抓取触发 (根据像素面积)
            area = (x2 - x1) * (y2 - y1)
            if area > (w * h * 0.4): # 如果物体占据画面40%面积，认为到位
                target_positions['gripper'] = 1  # 闭合夹爪
                print(">>> Target Reached! Gripping...")

            j2, j3 = inverse_kinematics(current_x, current_y)
            target_positions['shoulder_lift'], target_positions['elbow_flex'] = j2, j3

            # 可视化
            cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(annotated_frame, f"Ang:{int(angle)} Area:{area}", (x1, y1-10), 0, 0.6, (0,255,0), 2)
            break 

    cv2.imshow("SO100 YOLO-World Control", annotated_frame)
    if cv2.waitKey(1) & 0xFF == ord('q'): raise KeyboardInterrupt
    return current_x, current_y

# --- 主循环与初始化 ---
def main():
    cap = None
    try:
        from lerobot.robots.so100_follower import SO100Follower, SO100FollowerConfig
        from lerobot.teleoperators.keyboard import KeyboardTeleop, KeyboardTeleopConfig
        
        # 初始化
        model = YOLOWorld('/home/ljyyds/lerobot/yolov8s-world.pt') 
        target_input = input("Enter objects to catch (e.g., mouse, bottle): ").strip() or "mouse"
        target_objects = [obj.strip() for obj in target_input.split(',')]
        model.set_classes(["black box", "box"])

        cap = cv2.VideoCapture(1) # 你的相机索引
        robot = SO100Follower(SO100FollowerConfig(port="/dev/ttyACM0"))
        robot.connect()
        keyboard = KeyboardTeleop(KeyboardTeleopConfig())
        keyboard.connect()

        # 初始位姿
        target_positions = {k: 0.0 for k in ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']}
        current_x, current_y, pitch = 0.1629, 0.1131, 0.0

        print("System Ready. Press 'X' to exit.")
        while True:
            # 1. 视觉更新
            current_x, current_y = vision_control_update(target_positions, current_x, current_y, model, cap, target_objects)
            
            # 2. 键盘干预 (保留原有逻辑)
            k_act = keyboard.get_action()
            if k_act and 'x' in k_act: break
            
            # 3. 姿态补偿 (Wrist Flex 保持水平)
            target_positions['wrist_flex'] = -target_positions['shoulder_lift'] - target_positions['elbow_flex'] + pitch
            
            # 4. 执行动作 (P控制)
            obs = robot.get_observation()
            robot_action = {}
            for j_name, t_pos in target_positions.items():
                curr = apply_joint_calibration(j_name, obs[f"{j_name}.pos"])
                robot_action[f"{j_name}.pos"] = curr + 0.5 * (t_pos - curr)
            robot.send_action(robot_action)
            time.sleep(0.02)

    except Exception as e:
        traceback.print_exc()
    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()