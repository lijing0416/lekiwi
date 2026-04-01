import time
from lerobot.robots.lekiwi import LeKiwiClient, LeKiwiClientConfig
from lerobot.teleoperators.keyboard.teleop_keyboard import KeyboardTeleop, KeyboardTeleopConfig
from lerobot.utils.robot_utils import precise_sleep

FPS = 30

def main():
    # 1. 只配置机器人客户端和键盘
    robot_config = LeKiwiClientConfig(remote_ip="192.168.1.103", id="my_awesome_kiwi")
    keyboard_config = KeyboardTeleopConfig(id="my_laptop_keyboard")

    robot = LeKiwiClient(robot_config)
    keyboard = KeyboardTeleop(keyboard_config)

    # 2. 连接（此时 robot.connect() 只走网络，不占串口）
    robot.connect()
    keyboard.connect()

    if not robot.is_connected or not keyboard.is_connected:
        raise ValueError("Robot or keyboard is not connected!")

    print("Starting keyboard teleop loop...")
    while True:
        t0 = time.perf_counter()

        # 获取机器人状态
        observation = robot.get_observation()

        # 只获取键盘动作
        keyboard_keys = keyboard.get_action()
        # 转换为底座控制指令
        action = robot._from_keyboard_to_base_action(keyboard_keys)

        # 发送动作（如果没有按键，action 为空，不会有任何动作）
        if len(action) > 0:
            _ = robot.send_action(action)

        precise_sleep(max(1.0 / FPS - (time.perf_counter() - t0), 0.0))

if __name__ == "__main__":
    main()
