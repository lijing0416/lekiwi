import time
import math
import numpy as np
import cv2
import torch
import traceback
from lerobot.robots.so100_follower import SO100Follower, SO100FollowerConfig
from lerobot.teleoperators.keyboard import KeyboardTeleop, KeyboardTeleopConfig
from lerobot.motors import Motor, MotorNormMode

# ================= 1. 运动学逻辑 =================

def inverse_kinematics(x, y, l1=0.1159, l2=0.1350):
    theta1_offset = math.atan2(0.028, 0.11257)
    theta2_offset = math.atan2(0.0052, 0.1349) + theta1_offset
    r = math.sqrt(x**2 + y**2)
    r = np.clip(r, 0.05, l1 + l2)
    cos_theta2 = -(r**2 - l1**2 - l2**2) / (2 * l1 * l2)
    theta2 = math.pi - math.acos(np.clip(cos_theta2, -1, 1))
    theta1 = math.atan2(y, x) + math.atan2(l2 * math.sin(theta2), l1 + l2 * math.cos(theta2))
    return 90 - math.degrees(theta1 + theta1_offset), math.degrees(theta2 + theta2_offset) - 90

# ================= 2. 连续示范采集器 (双机位版) =================

def main():
    # 初始位姿
    HOME = {"x": 0.1629, "y": 0.1131, "pan": 0.0, "pitch": 0.0, "roll": 0.0, "gripper": 0.0}
    
    robot = None
    # 初始化双摄像头 (请根据实际情况调整索引 2 和 4)
    cap_top = cv2.VideoCapture(4)
    cap_bottom = cv2.VideoCapture(2)
    
    # 设置分辨率以保证帧率
    for c in [cap_top, cap_bottom]:
        c.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        c.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        c.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        c.set(cv2.CAP_PROP_FPS, 30)

    try:
        robot = SO100Follower(SO100FollowerConfig(port="/dev/ttyACM0"))
        chassis_ids = {"base_left_wheel": 7, "base_back_wheel": 8, "base_right_wheel": 9}
        for name, mid in chassis_ids.items():
            robot.bus.motors[name] = Motor(mid, "sts3215", MotorNormMode.RANGE_M100_100)
            robot.bus._id_to_model_dict[mid] = "sts3215"

        robot.connect(calibrate=False)
        keyboard = KeyboardTeleop(KeyboardTeleopConfig())
        keyboard.connect()

        curr = HOME.copy()
        dt = 1.0 / 25
        recording = False
        episode_data = []
        prev_state = None
        prev_action = np.zeros(6)

        print("\n[双机位采集模式启动]")
        print("R: 录制/停止 | X: 退出并保存")

        while True:
            start_time = time.perf_counter()
            
            # 同步抓取画面
            ret1, frame_top = cap_top.read()
            ret2, frame_bottom = cap_bottom.read()
            if not (ret1 and ret2):
                print("摄像头丢帧，请检查连接！")
                break

            act = keyboard.get_action()
            if act:
                if "x" in act: break
                if "r" in act:
                    if not recording:
                        recording = True
                        episode_data = []
                        prev_state = None
                        print(">>> 🔴 录制开始...")
                    else:
                        recording = False
                        if len(episode_data) > 20:
                            path = f"bc_dual_cam_{int(time.time())}.pt"
                            torch.save(episode_data, path)
                            print(f">>> ✅ 存档成功: {path}")
                        curr = HOME.copy()
                    time.sleep(0.3)

                # 控制逻辑
                step = 0.0015
                if "w" in act: curr["x"] += step
                if "s" in act: curr["x"] -= step
                if "e" in act: curr["y"] += step
                if "d" in act: curr["y"] -= step
                if "q" in act: curr["pan"] += 1.8
                if "a" in act: curr["pan"] -= 1.8
                if "t" in act: curr["roll"] += 5.0
                if "g" in act: curr["roll"] -= 5.0
                if "u" in act: curr["pitch"] += 1.8
                if "o" in act: curr["pitch"] -= 1.8
                if "y" in act: curr["gripper"] -= 10
                if "h" in act: curr["gripper"] += 10

            # 逆解与指令发送
            j2, j3 = inverse_kinematics(curr["x"], curr["y"])
            targets = {
                "shoulder_pan": curr["pan"],
                "shoulder_lift": j2,
                "elbow_flex": j3,
                "wrist_flex": -j2 - j3 + curr["pitch"],
                "wrist_roll": curr["roll"],
                "gripper": np.clip(curr["gripper"], -20, 100)
            }
            robot.bus.sync_write("Goal_Position", targets)

            # 数据向量
            curr_vec = np.array([curr["x"], curr["y"], curr["pan"], curr["pitch"], curr["roll"], curr["gripper"]])
            
            if recording:
                if prev_state is not None:
                    delta_action = curr_vec - prev_state
                    episode_data.append({
                        'observation': {
                            'image': cv2.resize(frame_top, (128, 128)),
                            'image_bottom': cv2.resize(frame_bottom, (128, 128)),
                            'state': prev_state.copy(),
                            'prev_action': prev_action.copy()
                        },
                        'action': delta_action.copy()
                    })
                    prev_action = delta_action.copy()
                prev_state = curr_vec.copy()

            # 双视窗拼接预览
            top_view = cv2.resize(frame_top, (320, 240))
            bot_view = cv2.resize(frame_bottom, (320, 240))
            display_img = np.hstack([top_view, bot_view])
            
            if recording:
                cv2.putText(display_img, "REC", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)
                cv2.putText(display_img, f"Steps: {len(episode_data)}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 1)
            
            cv2.imshow("Dual Camera Collection", display_img)
            cv2.waitKey(1)

            elapsed = time.perf_counter() - start_time
            if elapsed < dt:
                time.sleep(dt - elapsed)

    except Exception:
        traceback.print_exc()
    finally:
        if robot: robot.disconnect()
        cap_top.release()
        cap_bottom.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()