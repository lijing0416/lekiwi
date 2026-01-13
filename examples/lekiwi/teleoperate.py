import time
from lerobot.robots.lekiwi import LeKiwiClient, LeKiwiClientConfig
from lerobot.teleoperators.keyboard.teleop_keyboard import KeyboardTeleop, KeyboardTeleopConfig
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.visualization_utils import init_rerun, log_rerun_data

FPS = 30

def main():
    # 保持 ID 和 Host 一致
    robot_config = LeKiwiClientConfig(remote_ip="127.0.0.1", id="my_awesome_kiwi")
    keyboard_config = KeyboardTeleopConfig(id="my_laptop_keyboard")

    robot = LeKiwiClient(robot_config)
    keyboard = KeyboardTeleop(keyboard_config)

    robot.connect()
    keyboard.connect()

    init_rerun(session_name="lekiwi_teleop")

    if not robot.is_connected or not keyboard.is_connected:
        raise ValueError("Robot or keyboard is not connected!")

    print("Starting teleop loop...")
    print("控制提示: 请确保鼠标点击本终端，使用 W/A/S/D 控制。")

    while True:
        t0 = time.perf_counter()
        observation = robot.get_observation()
        
        # 获取键盘原始输入
        keyboard_keys = keyboard.get_action()
        
        # 转换动作
        base_action = robot._from_keyboard_to_base_action(keyboard_keys)
        
        # 初始化 action 防止后面 log_rerun_data 报错
        action = {}

        if len(base_action) > 0:
            action = {**base_action}
            # 【调试打印】如果这里有输出，说明客户端发送成功了
            print(f"发送动作: {action}")
            _ = robot.send_action(action)
        else:
            # 没按键时，不发送指令给底座（避免抖动）
            pass

        # 可视化（确保 action 无论如何都有定义）
        log_rerun_data(observation=observation, action=action)

        precise_sleep(max(1.0 / FPS - (time.perf_counter() - t0), 0.0))

if __name__ == "__main__":
    main()