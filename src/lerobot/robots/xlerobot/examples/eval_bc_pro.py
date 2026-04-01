import torch
import torch.nn as nn
import cv2
import numpy as np
import torchvision.transforms as T
import time
import math
import traceback
from collections import deque
from lerobot.robots.so100_follower import SO100Follower, SO100FollowerConfig
from lerobot.motors import Motor, MotorNormMode

# ================= 1. 9 通道双流模型 (必须匹配训练代码) =================
class DualCamPolicyNet(nn.Module):
    def __init__(self, state_dim=6, action_dim=6, window_size=3):
        super().__init__()
        def make_encoder():
            return nn.Sequential(
                nn.Conv2d(3 * window_size, 32, 5, stride=2), nn.ReLU(), # 3帧*3通道=9通道
                nn.Conv2d(32, 64, 3, stride=2), nn.ReLU(),
                nn.Conv2d(64, 128, 3, stride=2), nn.ReLU(),
                nn.AdaptiveAvgPool2d((4, 4)), nn.Flatten()
            )
        self.encoder_top = make_encoder()
        self.encoder_bot = make_encoder()
        # 2048(Top) + 2048(Bot) + 6(State) = 4102
        self.mlp = nn.Sequential(
            nn.Linear(4102, 512), nn.ReLU(),
            nn.Linear(512, 256), nn.ReLU(),
            nn.Linear(256, action_dim) 
        )

    def forward(self, img_top, img_bot, state):
        feat_top = self.encoder_top(img_top)
        feat_bot = self.encoder_bot(img_bot)
        combined = torch.cat([feat_top, feat_bot, state], dim=1)
        return self.mlp(combined)

# ================= 2. 运动学解算 =================
def inverse_kinematics(x, y, l1=0.1159, l2=0.1350):
    theta1_offset = math.atan2(0.028, 0.11257)
    theta2_offset = math.atan2(0.0052, 0.1349) + theta1_offset
    r = math.sqrt(x**2 + y**2)
    r = np.clip(r, 0.05, l1 + l2 - 0.001)
    cos_theta2 = -(r**2 - l1**2 - l2**2) / (2 * l1 * l2)
    theta2 = math.pi - math.acos(np.clip(cos_theta2, -1, 1))
    theta1 = math.atan2(y, x) + math.atan2(l2 * math.sin(theta2), l1 + l2 * math.cos(theta2))
    return 90 - math.degrees(theta1 + theta1_offset), math.degrees(theta2 + theta2_offset) - 90

# ================= 3. 推理主程序 =================
def main():
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"🚀 初始化硬件中... (设备: {device})")
    
    robot = None
    cap_top = cv2.VideoCapture(4)
    cap_bot = cv2.VideoCapture(2)
    
    # --- 核心新增：3 帧滑动窗口缓存 ---
    top_buf = deque(maxlen=3)
    bot_buf = deque(maxlen=3)
    
    for c in [cap_top, cap_bot]:
        c.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        c.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        c.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    try:
        robot = SO100Follower(SO100FollowerConfig(port="/dev/ttyACM0"))
        robot.connect(calibrate=False)
        
        # --- B. 载入模型 (确保文件名和 window_size=3) ---
        model = DualCamPolicyNet(window_size=3).to(device)
        model_path = "bc_temporal_last.pth" # 确认是 9 通道权重文件
        model.load_state_dict(torch.load(model_path, map_location=device))
        model.eval()
        print(f"🧠 9通道时间序模型 {model_path} 加载成功！")

        curr = {"x": 0.1629, "y": 0.1131, "pan": 0.0, "pitch": 0.0, "roll": 0.0, "gripper": 0.0}
        ACTION_STATS = np.array([0.002, 0.002, 2.0, 2.0, 5.0, 10.0])
        ACTION_GAIN = 5.0  # 时间序模型建议增益稍大一点

        transform = T.Compose([
            T.ToTensor(),
            T.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])

        print("\n⚡ 自动驾驶模式开启 (按 ESC 退出)")
        print(f"{'X':>7} | {'Y':>7} | {'Pan':>7} | {'Grip':>7} | {'AI_Mag'}")
        print("-" * 60)

        while True:
            start_loop = time.perf_counter()
            ret1, frame_top = cap_top.read()
            ret2, frame_bot = cap_bot.read()
            if not (ret1 and ret2): break

            # --- 核心逻辑：更新滑动窗口 ---
            t_img = transform(cv2.resize(frame_top, (128, 128)))
            b_img = transform(cv2.resize(frame_bot, (128, 128)))
            top_buf.append(t_img)
            bot_buf.append(b_img)

            # 只有当存满 3 帧才开始推理
            if len(top_buf) < 3:
                continue

            # 沿通道维度拼接: [3, 128, 128] * 3 -> [9, 128, 128]
            t_stack = torch.cat(list(top_buf), dim=0).unsqueeze(0).to(device)
            b_stack = torch.cat(list(bot_buf), dim=0).unsqueeze(0).to(device)
            
            state_vec = np.array([curr["x"], curr["y"], curr["pan"], curr["pitch"], curr["roll"], curr["gripper"]])
            t_state = torch.from_numpy(state_vec).float().unsqueeze(0).to(device)

            # AI 推理
            with torch.no_grad():
                pred_norm = model(t_stack, b_stack, t_state).cpu().numpy()[0]
                pred = pred_norm * ACTION_STATS 
            if int(time.perf_counter() * 10) % 2 == 0:
                mag = np.linalg.norm(pred_norm)
                print(f"\r{curr['x']:7.4f} | {curr['y']:7.4f} | {curr['pan']:7.1f} | {curr['gripper']:7.1f} | {mag:.4f}", end="")
            # 状态累积更新
            curr["x"] = np.clip(curr["x"] + pred[0] * ACTION_GAIN, 0, 0.35)
            curr["y"] = np.clip(curr["y"] + pred[1] * ACTION_GAIN, -0.6, 0.3)
            curr["pan"] = np.clip(curr["pan"] + pred[2] * ACTION_GAIN, -30, 25)
            curr["pitch"] = np.clip(curr["pitch"] + pred[3] * ACTION_GAIN, -90, 90)
            curr["roll"] = np.clip(curr["roll"] + pred[4] * ACTION_GAIN, -150, 150)
            curr["gripper"] = np.clip(curr["gripper"] + pred[5] * ACTION_GAIN, -20, 100)

            # 硬件指令下发
            try:

                j2, j3 = inverse_kinematics(curr["x"], curr["y"])
                targets = {
                    "shoulder_pan": curr["pan"],
                    "shoulder_lift": j2,
                    "elbow_flex": j3,
                    "wrist_flex": -j2 - j3 + curr["pitch"],
                    "wrist_roll": curr["roll"],
                    "gripper": curr["gripper"]
                }
                robot.bus.sync_write("Goal_Position", targets)
            except: pass

            vis = np.hstack([cv2.resize(frame_top, (320, 240)), cv2.resize(frame_bot, (320, 240))])
            cv2.imshow("Temporal Inference (9-Channel)", vis)
            if cv2.waitKey(1) == 27: break
            
            # 50Hz 频率控制
            dt = time.perf_counter() - start_loop
            if dt < 0.02: time.sleep(0.02 - dt)

    except Exception:
        traceback.print_exc()
    finally:
        print("\n🛑 停止运行并断开硬件")
        if robot: robot.disconnect()
        cap_top.release(); cap_bot.release(); cv2.destroyAllWindows()

if __name__ == "__main__":
    main()