#!/usr/bin/env python

import logging
import time
import json
from pathlib import Path

from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import (
    FeetechMotorsBus,
    OperatingMode,
)
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from ..teleoperator import Teleoperator
from .config_so101_leader import SO101LeaderConfig

logger = logging.getLogger(__name__)

class SO101Leader(Teleoperator):
    """
    SO-101 Leader Arm designed by TheRobotStudio and Hugging Face.
    Fixed version with persistent calibration.
    """

    config_class = SO101LeaderConfig
    name = "so101_leader"

    def __init__(self, config: SO101LeaderConfig):
        # 1. 路径初始化
        self.calibration_dir = Path.home() / ".cache/huggingface/lerobot/calibration/robots/lekiwi_client"
        self.calibration_dir.mkdir(parents=True, exist_ok=True)
        self.calibration_fpath = self.calibration_dir / f"{config.id}.json"
        
        # 2. 从磁盘加载现有的校准数据
        self.calibration = self._load_calibration()
        
        super().__init__(config)
        self.config = config
        
        norm_mode_body = MotorNormMode.DEGREES if config.use_degrees else MotorNormMode.RANGE_M100_100
        
        # 3. 将加载的校准传递给 Bus
        self.bus = FeetechMotorsBus(
            port=self.config.port,
            motors={
                "shoulder_pan": Motor(1, "sts3215", norm_mode_body),
                "shoulder_lift": Motor(2, "sts3215", norm_mode_body),
                "elbow_flex": Motor(3, "sts3215", norm_mode_body),
                "wrist_flex": Motor(4, "sts3215", norm_mode_body),
                "wrist_roll": Motor(5, "sts3215", norm_mode_body),
                "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
            },
            calibration=self.calibration,
        )

    def _load_calibration(self):
        """从 JSON 文件加载校准数据并转换为 MotorCalibration 对象"""
        if not self.calibration_fpath.exists():
            logger.info(f"No calibration file found at {self.calibration_fpath}")
            return None
        
        try:
            with open(self.calibration_fpath, "r") as f:
                data = json.load(f)
            
            calibration = {}
            for motor_name, m_data in data.items():
                # 如果是字典形式，转为 MotorCalibration 对象
                if isinstance(m_data, dict):
                    calibration[motor_name] = MotorCalibration(**m_data)
                else:
                    calibration[motor_name] = m_data
            logger.info(f"Successfully loaded calibration from {self.calibration_fpath}")
            return calibration
        except Exception as e:
            logger.error(f"Failed to load calibration file: {e}")
            return None

    def _save_calibration(self):
        """将当前的校准对象序列化并保存到磁盘"""
        if not self.calibration:
            return
            
        # 转换对象为可序列化的字典
        serializable_calib = {}
        for motor_name, calib in self.calibration.items():
            if hasattr(calib, "__dict__"):
                serializable_calib[motor_name] = calib.__dict__
            else:
                serializable_calib[motor_name] = calib
                
        with open(self.calibration_fpath, "w") as f:
            json.dump(serializable_calib, f, indent=4)
        logger.info(f"Calibration successfully saved to {self.calibration_fpath}")

    @property
    def action_features(self) -> dict[str, type]:
        return {f"{motor}.pos": float for motor in self.bus.motors}

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        self.bus.connect()

        # 逻辑：如果磁盘加载了校准，直接同步到电机，跳过手动校准
        if self.calibration is not None and calibrate:
            logger.info(f"Found saved calibration for {self.config.id}. Applying to motors...")
            self.bus.write_calibration(self.calibration)
        elif not self.is_calibrated and calibrate:
            logger.info("No valid calibration found in motors or disk. Starting manual calibration...")
            self.calibrate()

        self.configure()
        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        return self.bus.is_calibrated

    def calibrate(self) -> None:
        if self.calibration:
            user_input = input(
                f"Press ENTER to use provided calibration file associated with the id {self.config.id}, or type 'c' and press ENTER to run calibration: "
            )
            if user_input.strip().lower() != "c":
                logger.info(f"Writing calibration file associated with the id {self.config.id} to the motors")
                self.bus.write_calibration(self.calibration)
                return

        logger.info(f"\nRunning calibration of {self}")
        self.bus.disable_torque()
        for motor in self.bus.motors:
            self.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)

        input(f"Move {self} to the middle of its range of motion and press ENTER....")
        homing_offsets = self.bus.set_half_turn_homings()

        print(
            "Move all joints sequentially through their entire ranges "
            "of motion.\nRecording positions. Press ENTER to stop..."
        )
        range_mins, range_maxes = self.bus.record_ranges_of_motion()

        self.calibration = {}
        for motor, m in self.bus.motors.items():
            self.calibration[motor] = MotorCalibration(
                id=m.id,
                drive_mode=0,
                homing_offset=homing_offsets[motor],
                range_min=range_mins[motor],
                range_max=range_maxes[motor],
            )

        # 核心修复：写入电机 + 写入磁盘
        self.bus.write_calibration(self.calibration)
        self._save_calibration()

    def configure(self) -> None:
        self.bus.disable_torque()
        self.bus.configure_motors()
        for motor in self.bus.motors:
            self.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)

    def setup_motors(self) -> None:
        for motor in reversed(self.bus.motors):
            input(f"Connect the controller board to the '{motor}' motor only and press enter.")
            self.bus.setup_motor(motor)
            print(f"'{motor}' motor id set to {self.bus.motors[motor].id}")

    def get_action(self) -> dict[str, float]:
        start = time.perf_counter()
        action = self.bus.sync_read("Present_Position")
        action = {f"{motor}.pos": val for motor, val in action.items()}
        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read action: {dt_ms:.1f}ms")
        return action

    def send_feedback(self, feedback: dict[str, float]) -> None:
        raise NotImplementedError

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        self.bus.disconnect()
        logger.info(f"{self} disconnected.")