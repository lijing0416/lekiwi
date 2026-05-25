#!/usr/bin/env python3
"""
VR control for XLerobot robot
Uses handle_vr_input with delta action control
"""

# Standard library imports
import argparse
import asyncio
from functools import cached_property
import glob
import logging
import math
import os
import sys
import threading
import time
import traceback
from typing import Any
from pathlib import Path

# Third-party imports
import pygame

# Add XleVR path to allow vr_monitor import
XLEVR_PATH = str(Path(__file__).parent.parent / "XleVR")
if XLEVR_PATH not in sys.path:
    sys.path.insert(0, XLEVR_PATH)

# Local imports
from vr_monitor import VRMonitor
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode
from lerobot.robots.xlerobot.src.robots.xlerobot.config_xlerobot import XLerobotConfig
from lerobot.robots.robot import Robot
from lerobot.robots.utils import ensure_safe_goal_position
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.robots.xlerobot.src.model.SO101Robot import SO101Kinematics

# Setup logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Joint mapping configurations
RIGHT_JOINT_MAP = {
    "shoulder_pan": "right_arm_shoulder_pan",
    "shoulder_lift": "right_arm_shoulder_lift",
    "elbow_flex": "right_arm_elbow_flex",
    "wrist_flex": "right_arm_wrist_flex",
    "wrist_roll": "right_arm_wrist_roll",
    "gripper": "right_arm_gripper",
}


class SingleRightArmXLerobot(Robot):
    """XLerobot-compatible robot wrapper that connects only the right arm bus."""

    config_class = XLerobotConfig
    name = "xlerobot"

    def __init__(self, config: XLerobotConfig, port: str):
        super().__init__(config)
        self.config = config
        self.port = port
        norm_mode = MotorNormMode.DEGREES if config.use_degrees else MotorNormMode.RANGE_M100_100
        calibration = {
            name: self.calibration[name]
            for name in RIGHT_JOINT_MAP.values()
            if name in self.calibration
        }
        self.bus = FeetechMotorsBus(
            port=port,
            motors={
                "right_arm_shoulder_pan": Motor(1, "sts3215", norm_mode),
                "right_arm_shoulder_lift": Motor(2, "sts3215", norm_mode),
                "right_arm_elbow_flex": Motor(3, "sts3215", norm_mode),
                "right_arm_wrist_flex": Motor(4, "sts3215", norm_mode),
                "right_arm_wrist_roll": Motor(5, "sts3215", norm_mode),
                "right_arm_gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
            },
            calibration=calibration,
        )
        self.right_arm_motors = list(self.bus.motors)

    @cached_property
    def observation_features(self) -> dict[str, type]:
        return dict.fromkeys((f"{name}.pos" for name in self.right_arm_motors), float)

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self.observation_features

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected

    @property
    def is_calibrated(self) -> bool:
        if not self.bus.calibration:
            return False
        try:
            return self.bus.is_calibrated
        except Exception:
            return False

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        self.bus.connect()

        missing_calibration = [name for name in self.right_arm_motors if name not in self.bus.calibration]
        if missing_calibration:
            if not calibrate:
                raise RuntimeError(
                    "No complete right-arm calibration found. Missing: "
                    f"{missing_calibration}. Run once without --no-calibrate to create it."
                )
            self.calibrate()
        else:
            try:
                self.bus.write_calibration(self.bus.calibration)
                logger.info("Right-arm calibration restored from %s", self.calibration_fpath)
            except Exception as e:
                logger.warning("Failed to write saved calibration to motors: %s", e)
                if calibrate:
                    self.calibrate()

        self.configure()
        logger.info("%s connected on %s.", self, self.port)

    def calibrate(self) -> None:
        logger.info("Running calibration for right arm only")
        self.bus.disable_torque()
        for name in self.right_arm_motors:
            self.bus.write("Operating_Mode", name, OperatingMode.POSITION.value)

        input("Move right arm motors to the middle of their range of motion and press ENTER....")
        homing_offsets = self.bus.set_half_turn_homings(self.right_arm_motors)

        print(
            "Move all right arm joints sequentially through their entire ranges of motion.\n"
            "Recording positions. Press ENTER to stop..."
        )
        range_mins, range_maxes = self.bus.record_ranges_of_motion(self.right_arm_motors)

        calibration = {
            name: MotorCalibration(
                id=motor.id,
                drive_mode=0,
                homing_offset=homing_offsets[name],
                range_min=range_mins[name],
                range_max=range_maxes[name],
            )
            for name, motor in self.bus.motors.items()
        }
        self.bus.write_calibration(calibration)
        self.calibration.update(calibration)
        self._save_calibration()
        print("Calibration saved to", self.calibration_fpath)

    def configure(self) -> None:
        self.bus.disable_torque()
        self.bus.configure_motors()
        for name in self.right_arm_motors:
            self.bus.write("Operating_Mode", name, OperatingMode.POSITION.value)
            self.bus.write("P_Coefficient", name, 16)
            self.bus.write("I_Coefficient", name, 0)
            self.bus.write("D_Coefficient", name, 43)
        self.bus.enable_torque()

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        arm_pos = self.bus.sync_read("Present_Position", self.right_arm_motors)
        return {f"{name}.pos": value for name, value in arm_pos.items()}

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        right_arm_pos = {
            key: value
            for key, value in action.items()
            if key.startswith("right_arm_") and key.endswith(".pos")
        }

        if self.config.max_relative_target is not None and right_arm_pos:
            present_pos = self.bus.sync_read("Present_Position", self.right_arm_motors)
            safe_goal_pos = ensure_safe_goal_position(
                {
                    key.replace(".pos", ""): (goal, present_pos[key.replace(".pos", "")])
                    for key, goal in right_arm_pos.items()
                },
                self.config.max_relative_target,
            )
            right_arm_pos = {f"{key}.pos": value for key, value in safe_goal_pos.items()}

        right_arm_pos_raw = {key.replace(".pos", ""): value for key, value in right_arm_pos.items()}
        if right_arm_pos_raw:
            self.bus.sync_write("Goal_Position", right_arm_pos_raw)
        return right_arm_pos

    def disconnect(self):
        if not self.is_connected:
            return
        self.bus.disconnect(self.config.disable_torque_on_disconnect)
        logger.info("%s disconnected.", self)

# Joint calibration coefficients - manually edit
# Format: [joint_name, zero_position_offset(degrees), scale_factor]
JOINT_CALIBRATION = [
    ['shoulder_pan', 6.0, 1.0],      # Joint1: zero position offset, scale factor
    ['shoulder_lift', 2.0, 0.97],     # Joint2: zero position offset, scale factor
    ['elbow_flex', 0.0, 1.05],        # Joint3: zero position offset, scale factor
    ['wrist_flex', 0.0, 0.94],        # Joint4: zero position offset, scale factor
    ['wrist_roll', 0.0, 0.5],        # Joint5: zero position offset, scale factor
    ['gripper', 0.0, 1.0],           # Joint6: zero position offset, scale factor
]


class SimpleTeleopArm:
    """
    A class for controlling a robot arm using VR input with delta action control.
    
    This class provides inverse kinematics-based arm control with proportional control
    for smooth movement and gripper operations based on VR controller input.
    """
    
    def __init__(self, joint_map, initial_obs, kinematics, prefix="right", kp=1):
        self.joint_map = joint_map
        self.prefix = prefix
        self.kp = kp
        self.kinematics = kinematics
        
        # Initial joint positions - adapted for XLerobot observation format
        self.joint_positions = {
            "shoulder_pan": initial_obs[f"{prefix}_arm_shoulder_pan.pos"],
            "shoulder_lift": initial_obs[f"{prefix}_arm_shoulder_lift.pos"],
            "elbow_flex": initial_obs[f"{prefix}_arm_elbow_flex.pos"],
            "wrist_flex": initial_obs[f"{prefix}_arm_wrist_flex.pos"],
            "wrist_roll": initial_obs[f"{prefix}_arm_wrist_roll.pos"],
            "gripper": initial_obs[f"{prefix}_arm_gripper.pos"],
        }
        
        # Set initial x/y to fixed values
        self.current_x = 0.1629
        self.current_y = 0.1131
        self.pitch = 0.0
        
        # Delta control state variables for VR input
        self.last_vr_time = 0.0
        self.vr_deadzone = 0.001  # Minimum movement threshold
        self.max_delta_per_frame = 0.005  # Maximum position change per frame
        
        # Set step size
        self.degree_step = 2
        self.xy_step = 0.005
        
        # P control target positions, set to zero position
        self.target_positions = {
            "shoulder_pan": 0.0,
            "shoulder_lift": 0.0,
            "elbow_flex": 0.0,
            "wrist_flex": 0.0,
            "wrist_roll": 0.0,
            "gripper": 0.0,
        }
        self.zero_pos = {
            'shoulder_pan': 0.0,
            'shoulder_lift': 0.0,
            'elbow_flex': 0.0,
            'wrist_flex': 0.0,
            'wrist_roll': 0.0,
            'gripper': 0.0
        }

    def move_to_zero_position(self, robot):
        print(f"[{self.prefix}] Moving to Zero Position: {self.zero_pos} ......")
        self.target_positions = self.zero_pos.copy()
        
        # Reset kinematics variables to initial state
        self.current_x = 0.1629
        self.current_y = 0.1131
        self.pitch = 0.0
        
        # Reset delta control state
        self.last_vr_time = 0.0
        
        # Explicitly set wrist_flex
        self.target_positions["wrist_flex"] = 0.0
        
        action = self.p_control_action(robot)
        robot.send_action(action)

    def handle_vr_input(self, vr_goal, gripper_state):
        """
        Handle VR input with delta action control - incremental position updates.
        
        Args:
            vr_goal: VR controller goal data containing target position and orientations
            gripper_state: Current gripper state (not used in current implementation)
        """
        if vr_goal is None:
            return
        
        # VR goal contains: target_position [x, y, z], wrist_roll_deg, wrist_flex_deg, gripper_closed
        if not hasattr(vr_goal, 'target_position') or vr_goal.target_position is None:
            return
            
        # Extract VR position data
        # Get current VR position
        current_vr_pos = vr_goal.target_position  # [x, y, z] in meters
        
        # Initialize previous VR position if not set
        if not hasattr(self, 'prev_vr_pos'):
            self.prev_vr_pos = current_vr_pos
            return  # Skip first frame to establish baseline
        
        # Calculate relative change (delta) from previous frame
        vr_x = (current_vr_pos[0] - self.prev_vr_pos[0]) * 100 # Scale for the shoulder
        vr_y = (current_vr_pos[1] - self.prev_vr_pos[1]) * 50 
        vr_z = (current_vr_pos[2] - self.prev_vr_pos[2]) * 15

        # print(f'vr_x: {vr_x}, vr_y: {vr_y}, vr_z: {vr_z}')

        # Update previous position for next frame
        self.prev_vr_pos = current_vr_pos
        
        # Delta control parameters - adjust these for sensitivity
        pos_scale = 0.01  # Position sensitivity scaling
        angle_scale = 1.0  # Angle sensitivity scaling
        delta_limit = 0.01  # Maximum delta per update (meters)
        angle_limit = 8.0  # Maximum angle delta per update (degrees)
        
        delta_x = vr_x * pos_scale
        delta_y = vr_y * pos_scale  
        delta_z = vr_z * pos_scale
        
        # Limit delta values to prevent sudden movements
        delta_x = max(-delta_limit, min(delta_limit, delta_x))
        delta_y = max(-delta_limit, min(delta_limit, delta_y))
        delta_z = max(-delta_limit, min(delta_limit, delta_z))
        
        self.current_x += -delta_y  # VR Y is forward/backward in XleVR
        self.current_y += delta_z  # VR Z is up/down in XleVR

        # Handle wrist angles with delta control - use relative changes
        if hasattr(vr_goal, 'wrist_flex_deg') and vr_goal.wrist_flex_deg is not None:
            # Initialize previous wrist_flex if not set
            if not hasattr(self, 'prev_wrist_flex'):
                self.prev_wrist_flex = vr_goal.wrist_flex_deg
                return
            
            # Calculate relative change from previous frame
            delta_pitch = (vr_goal.wrist_flex_deg - self.prev_wrist_flex) * angle_scale
            delta_pitch = max(-angle_limit, min(angle_limit, delta_pitch))
            self.pitch -= delta_pitch
            self.pitch = max(-90, min(90, self.pitch))  # Limit pitch range
            
            # Update previous value for next frame
            self.prev_wrist_flex = vr_goal.wrist_flex_deg
        
        if hasattr(vr_goal, 'wrist_roll_deg') and vr_goal.wrist_roll_deg is not None:
            # Initialize previous wrist_roll if not set
            if not hasattr(self, 'prev_wrist_roll'):
                self.prev_wrist_roll = vr_goal.wrist_roll_deg
                return
            
            delta_roll = (vr_goal.wrist_roll_deg - self.prev_wrist_roll) * angle_scale
            delta_roll = max(-angle_limit, min(angle_limit, delta_roll))
            
            current_roll = self.target_positions.get("wrist_roll", 0.0)
            new_roll = current_roll + delta_roll
            new_roll = max(-90, min(90, new_roll))  # Limit roll range
            self.target_positions["wrist_roll"] = new_roll
            
            # Update previous value for next frame
            self.prev_wrist_roll = vr_goal.wrist_roll_deg
        
        # VR Z axis controls shoulder_pan joint (delta control)
        if abs(delta_x) > 0.001:  # Only update if significant movement
            x_scale = 200.0  # Reduced scaling factor for delta control
            delta_pan = delta_x * x_scale
            delta_pan = max(-angle_limit, min(angle_limit, delta_pan))
            current_pan = self.target_positions.get("shoulder_pan", 0.0)
            new_pan = current_pan + delta_pan
            new_pan = max(-180, min(180, new_pan))  # Limit pan range
            self.target_positions["shoulder_pan"] = new_pan
        
        try:
            joint2_target, joint3_target = self.kinematics.inverse_kinematics(self.current_x, self.current_y)
            # Smooth transition to new joint positions,  Smoothing factor 0-1, lower = smoother
            alpha = 0.1
            self.target_positions["shoulder_lift"] = (1-alpha) * self.target_positions.get("shoulder_lift", 0.0) + alpha * joint2_target
            self.target_positions["elbow_flex"] = (1-alpha) * self.target_positions.get("elbow_flex", 0.0) + alpha * joint3_target
        except Exception as e:
            print(f"[{self.prefix}] VR IK failed: {e}")
        
        # Calculate wrist_flex to maintain end-effector orientation
        self.target_positions["wrist_flex"] = (-self.target_positions["shoulder_lift"] - 
                                               self.target_positions["elbow_flex"] + self.pitch)
   
        # Handle gripper state directly
        if vr_goal.metadata.get('trigger', 0) > 0.5:
            self.target_positions["gripper"] = 45
        else:
            self.target_positions["gripper"] = 0.0

    def p_control_action(self, robot):
        """
        Generate proportional control action based on target positions.
        
        Args:
            robot: Robot instance to get current observations
            
        Returns:
            dict: Action dictionary with position commands for each joint
        """
        obs = robot.get_observation()
        current = {j: obs[f"{self.prefix}_arm_{j}.pos"] for j in self.joint_map}
        action = {}
        for j in self.target_positions:
            error = self.target_positions[j] - current[j]
            control = self.kp * error
            action[f"{self.joint_map[j]}.pos"] = current[j] + control
        return action


def resolve_arm_port(requested_port: str | None, fallback_port: str) -> str:
    if requested_port:
        return requested_port

    candidates = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
    if fallback_port in candidates:
        return fallback_port
    if len(candidates) == 1:
        print(f"[MAIN] Auto-selected single detected serial port: {candidates[0]}")
        return candidates[0]

    print(f"[MAIN] Using default right-arm port: {fallback_port}")
    if candidates:
        print(f"[MAIN] Available serial ports: {', '.join(candidates)}")
    return fallback_port


def parse_args():
    parser = argparse.ArgumentParser(description="Control one XLerobot arm with the VR right controller.")
    parser.add_argument(
        "--port",
        default=os.environ.get("XLEROBOT_ARM_PORT"),
        help="Serial port for the right arm. Defaults to XLEROBOT_ARM_PORT, then robot.port2.",
    )
    parser.add_argument(
        "--robot-id",
        default=None,
        help="Robot id used for loading/saving calibration. Defaults to the existing XLerobotConfig id.",
    )
    parser.add_argument(
        "--no-calibrate",
        action="store_true",
        help="Do not run manual calibration if a right-arm calibration file is missing.",
    )
    parser.add_argument(
        "--use-degrees",
        action="store_true",
        help="Use degree-normalized joint positions instead of the default -100..100 range.",
    )
    parser.add_argument(
        "--elbow-min-deg",
        type=float,
        default=-90.0,
        help="Minimum internal elbow IK angle in degrees. Lower values increase forward reach.",
    )
    return parser.parse_args()


def main():
    """
    Main function for VR teleoperation of XLerobot.
    
    Initializes the robot connection, VR monitoring, and runs the main control loop
    for one-arm robot control with the VR right controller.
    """
    args = parse_args()
    print("XLerobot Single-Arm VR Control Example")
    print("="*50)
    
    # Initialize pygame for keyboard input handling
    pygame.init()
    robot = None

    try:
        robot_config = XLerobotConfig(id=args.robot_id, use_degrees=args.use_degrees)
        arm_port = resolve_arm_port(args.port, robot_config.port2)
        robot = SingleRightArmXLerobot(robot_config, arm_port)
        
        try:
            robot.connect(calibrate=not args.no_calibrate)
            print(f"[MAIN] Successfully connected to right arm on {arm_port}")
            if robot.is_calibrated:
                print(f"[MAIN] Right arm is calibrated and ready to use!")
            else:
                print(f"[MAIN] Right arm requires calibration")
        except Exception as e:
            print(f"[MAIN] Failed to connect to right arm: {e}")
            print(f"[MAIN] Robot config: {robot_config}")
            print(f"[MAIN] Robot: {robot}")
            print("[MAIN] Tip: pass the correct port, for example: --port /dev/ttyACM0")
            return
        
        # Initialize VR monitor
        print("🔧 Initializing VR monitor...")
        vr_monitor = VRMonitor()
        if not vr_monitor.initialize():
            print("❌ VR monitor initialization failed")
            return
        print("🚀 Starting VR monitoring...")
        vr_thread = threading.Thread(target=lambda: asyncio.run(vr_monitor.start_monitoring()), daemon=True)
        vr_thread.start()
        print("✅ VR system ready")

        # Init the right arm controller. VR left controller, head, and base are intentionally unused.
        obs = robot.get_observation()
        kin_right = SO101Kinematics(joint3_limits=(math.radians(args.elbow_min_deg), math.pi))
        right_arm = SimpleTeleopArm(RIGHT_JOINT_MAP, obs, kin_right, prefix="right")

        # Move the controlled arm to zero position at start
        right_arm.move_to_zero_position(robot)
        
        # Main VR control loop
        print("Starting right-controller VR control loop. Press ESC to exit.")
        try:
            while True:
                # Get VR controller data
                dual_goals = vr_monitor.get_latest_goal_nowait()
                right_goal = dual_goals.get("right") if dual_goals else None

                # Wait for VR connection before proceeding
                if dual_goals is None:
                    time.sleep(0.01)  # Wait 10ms for VR connection
                    continue

                # Handle VR input for the right arm only
                right_arm.handle_vr_input(right_goal, gripper_state=None)
                
                action = right_arm.p_control_action(robot)
                robot.send_action(action)
                
                # Handle keyboard exit (press ESC to quit)
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        print("Quit event detected, exiting...")
                        break
                    elif event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_ESCAPE:
                            print("ESC pressed, exiting...")
                            break
                else:
                    continue  # Continue the while loop if no break occurred
                break  # Break the while loop if a break occurred in the for loop
                
        finally:
            if robot is not None:
                robot.disconnect()
            print("VR teleoperation ended.")
        
    except Exception as e:
        print(f"Program execution failed: {e}")
        traceback.print_exc()
        
    finally:
        # Cleanup
        try:
            pygame.quit()
        except:
            pass
        try:
            if robot is not None:
                robot.disconnect()
        except:
            pass

if __name__ == "__main__":
    main()
