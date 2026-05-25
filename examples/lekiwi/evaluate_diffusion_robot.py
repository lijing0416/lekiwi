# !/usr/bin/env python

import argparse
import csv
import time
from contextlib import nullcontext
from copy import copy
from dataclasses import replace
from pathlib import Path
from typing import Any

import cv2
import torch

import lerobot.scripts.lerobot_record as lerobot_record
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.utils import hw_to_dataset_features
from lerobot.policies.diffusion.modeling_diffusion import DiffusionPolicy
from lerobot.policies.factory import make_pre_post_processors
from lerobot.policies.pretrained import PreTrainedPolicy
from lerobot.policies.utils import prepare_observation_for_inference
from lerobot.processor import PolicyAction, PolicyProcessorPipeline, make_default_processors
from lerobot.robots.lekiwi import LeKiwiClient, LeKiwiClientConfig
from lerobot.utils.constants import ACTION, OBS_STATE, OBS_STR
from lerobot.utils.control_utils import init_keyboard_listener
from lerobot.utils.utils import log_say
from lerobot.utils.visualization_utils import init_rerun
from tensorrt_diffusion import attach_trt_diffusion_backend

NUM_EPISODES = 1
FPS = 20
EPISODE_TIME_SEC = 60
RESET_TIME_SEC = 30
IMAGE_SIZE = 224
NUM_INFERENCE_STEPS = 16
TASK_DESCRIPTION = "Grab the lego brick"
DEFAULT_MODEL_ID = (
    "/home/ljyyds/lerobot/outputs/train/train/"
    "diffusion_aloha_merged_20260517_11datasets_bs32_aug/"
    "checkpoints/050000/pretrained_model"
)
DEFAULT_DATASET_ID = "ljyyds/diffusion_eval_test"
DEFAULT_REMOTE_IP = "192.168.0.131"


class ResizeCameraObservationWrapper:
    """Resize camera observations before inference and dataset recording."""

    def __init__(self, robot: LeKiwiClient, image_size: int):
        self._robot = robot
        self._image_size = image_size
        self._camera_keys = tuple(robot.config.cameras)

    def __getattr__(self, name):
        return getattr(self._robot, name)

    def get_observation(self):
        obs = self._robot.get_observation()
        for camera_key in self._camera_keys:
            frame = obs.get(camera_key)
            if frame is None or frame.shape[:2] == (self._image_size, self._image_size):
                continue
            obs[camera_key] = cv2.resize(
                frame,
                (self._image_size, self._image_size),
                interpolation=cv2.INTER_AREA,
            )
        return obs


def _percentile(values: list[float], q: float) -> float:
    if not values:
        return 0.0
    values = sorted(values)
    idx = min(len(values) - 1, max(0, int(round((len(values) - 1) * q))))
    return values[idx]


def _diffusion_action_queue(policy: PreTrainedPolicy):
    queues = getattr(policy, "_queues", None)
    if isinstance(queues, dict):
        return queues.get(ACTION)
    return None


class LatencyMonitor:
    """Collects and prints deployment latency in milliseconds."""

    def __init__(self, fps: int, print_every: int, csv_path: str | None = None):
        self.fps = fps
        self.print_every = print_every
        self.budget_ms = 1000.0 / fps
        self.csv_path = Path(csv_path).expanduser() if csv_path else None
        self.rows: list[dict[str, float | int]] = []
        self.prepare_ms: list[float] = []
        self.preprocess_ms: list[float] = []
        self.policy_ms: list[float] = []
        self.postprocess_ms: list[float] = []
        self.model_total_ms: list[float] = []
        self.observation_ms: list[float] = []
        self.send_action_ms: list[float] = []
        self.policy_calls = 0
        self.full_forward_calls = 0

    def record_observation(self, ms: float) -> None:
        self.observation_ms.append(ms)

    def record_send_action(self, ms: float) -> None:
        self.send_action_ms.append(ms)

    def record_policy(
        self,
        *,
        prepare_ms: float,
        preprocess_ms: float,
        policy_ms: float,
        postprocess_ms: float,
        total_ms: float,
        is_full_forward: bool,
        queued_actions_after: int | None,
    ) -> None:
        self.policy_calls += 1
        if is_full_forward:
            self.full_forward_calls += 1
        self.prepare_ms.append(prepare_ms)
        self.preprocess_ms.append(preprocess_ms)
        self.policy_ms.append(policy_ms)
        self.postprocess_ms.append(postprocess_ms)
        self.model_total_ms.append(total_ms)
        self.rows.append(
            {
                "policy_call": self.policy_calls,
                "full_forward": int(is_full_forward),
                "queued_actions_after": queued_actions_after if queued_actions_after is not None else -1,
                "prepare_ms": prepare_ms,
                "preprocess_ms": preprocess_ms,
                "policy_ms": policy_ms,
                "postprocess_ms": postprocess_ms,
                "model_total_ms": total_ms,
            }
        )

        if self.print_every > 0 and self.policy_calls % self.print_every == 0:
            self.print_live_summary()

    def _avg(self, values: list[float]) -> float:
        return sum(values) / len(values) if values else 0.0

    def _latest(self, values: list[float]) -> float:
        return values[-1] if values else 0.0

    def print_live_summary(self) -> None:
        full_forward_ratio = self.full_forward_calls / max(1, self.policy_calls)
        print(
            "[Latency] "
            f"calls={self.policy_calls} full_forward={self.full_forward_calls} "
            f"({full_forward_ratio:.1%}) | "
            f"latest model={self._latest(self.model_total_ms):.1f}ms "
            f"(prepare {self._latest(self.prepare_ms):.1f}, "
            f"pre {self._latest(self.preprocess_ms):.1f}, "
            f"policy {self._latest(self.policy_ms):.1f}, "
            f"post {self._latest(self.postprocess_ms):.1f}) | "
            f"avg model={self._avg(self.model_total_ms):.1f}ms "
            f"p95={_percentile(self.model_total_ms, 0.95):.1f}ms | "
            f"fps budget={self.budget_ms:.1f}ms"
        )

    def print_final_summary(self) -> None:
        if not self.model_total_ms and not self.observation_ms and not self.send_action_ms:
            print("[Latency] No latency samples were collected.")
            return

        def line(label: str, values: list[float]) -> str:
            return (
                f"{label:<18} avg={self._avg(values):7.2f}ms "
                f"p95={_percentile(values, 0.95):7.2f}ms "
                f"max={(max(values) if values else 0.0):7.2f}ms "
                f"n={len(values)}"
            )

        print("\n========== Latency Summary ==========")
        print(f"Target fps: {self.fps} | per-frame budget: {self.budget_ms:.2f} ms")
        print(
            f"Policy calls: {self.policy_calls} | "
            f"Diffusion full forward calls: {self.full_forward_calls}"
        )
        print(line("get_observation", self.observation_ms))
        print(line("prepare_obs", self.prepare_ms))
        print(line("preprocessor", self.preprocess_ms))
        print(line("policy_select", self.policy_ms))
        print(line("postprocessor", self.postprocess_ms))
        print(line("model_total", self.model_total_ms))
        print(line("send_action", self.send_action_ms))
        print("=====================================\n")

    def save_csv(self) -> None:
        if self.csv_path is None or not self.rows:
            return
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        with self.csv_path.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(self.rows[0].keys()))
            writer.writeheader()
            writer.writerows(self.rows)
        print(f"[Latency] CSV saved to: {self.csv_path}")


class TimedRobot:
    """Small proxy that times robot I/O without changing the robot implementation."""

    def __init__(self, robot: LeKiwiClient, monitor: LatencyMonitor):
        self._robot = robot
        self._monitor = monitor

    def __getattr__(self, name):
        return getattr(self._robot, name)

    def get_observation(self):
        t0 = time.perf_counter()
        obs = self._robot.get_observation()
        self._monitor.record_observation((time.perf_counter() - t0) * 1000.0)
        return obs

    def send_action(self, action):
        t0 = time.perf_counter()
        sent_action = self._robot.send_action(action)
        self._monitor.record_send_action((time.perf_counter() - t0) * 1000.0)
        return sent_action


class ActionFilter:
    """Smooth and slow policy actions in physical units before sending to the robot."""

    def __init__(
        self,
        action_names: list[str],
        state_names: list[str],
        *,
        ema_alpha: float,
        arm_action_scale: float,
        base_velocity_scale: float,
        max_arm_delta: float | None,
    ):
        self.action_names = action_names
        self.state_names = state_names
        self.ema_alpha = ema_alpha
        self.arm_action_scale = arm_action_scale
        self.base_velocity_scale = base_velocity_scale
        self.max_arm_delta = max_arm_delta
        self.previous_action: torch.Tensor | None = None
        self.state_name_to_idx = {name: i for i, name in enumerate(state_names)}
        self.arm_position_indices = [
            i for i, name in enumerate(action_names) if name.endswith(".pos") and "gripper" not in name
        ]
        self.base_velocity_indices = [i for i, name in enumerate(action_names) if name.endswith(".vel")]

    def reset(self) -> None:
        self.previous_action = None

    @property
    def enabled(self) -> bool:
        return (
            self.ema_alpha < 1.0
            or self.arm_action_scale < 1.0
            or self.base_velocity_scale < 1.0
            or self.max_arm_delta is not None
        )

    def _current_action_like_state(self, observation: dict[str, Any], action: torch.Tensor) -> torch.Tensor | None:
        if OBS_STATE not in observation:
            return None
        state = torch.as_tensor(observation[OBS_STATE], dtype=action.dtype, device=action.device).flatten()
        current = action.clone()
        found = False
        for action_idx, name in enumerate(self.action_names):
            state_idx = self.state_name_to_idx.get(name)
            if state_idx is None:
                continue
            current[action_idx] = state[state_idx]
            found = True
        return current if found else None

    def __call__(self, action: PolicyAction, observation: dict[str, Any]) -> PolicyAction:
        if not self.enabled:
            return action

        original_shape = action.shape
        values = action.detach().clone().reshape(-1)
        current = self._current_action_like_state(observation, values)

        if current is not None and self.arm_action_scale < 1.0:
            idx = torch.as_tensor(self.arm_position_indices, dtype=torch.long, device=values.device)
            values[idx] = current[idx] + self.arm_action_scale * (values[idx] - current[idx])

        if self.base_velocity_scale < 1.0:
            idx = torch.as_tensor(self.base_velocity_indices, dtype=torch.long, device=values.device)
            values[idx] = values[idx] * self.base_velocity_scale

        if self.previous_action is None:
            self.previous_action = current.clone() if current is not None else values.clone()

        if self.max_arm_delta is not None:
            idx = torch.as_tensor(self.arm_position_indices, dtype=torch.long, device=values.device)
            prev = self.previous_action.to(values.device)
            values[idx] = torch.clamp(
                values[idx],
                min=prev[idx] - self.max_arm_delta,
                max=prev[idx] + self.max_arm_delta,
            )

        if self.ema_alpha < 1.0:
            prev = self.previous_action.to(values.device)
            values = prev + self.ema_alpha * (values - prev)

        self.previous_action = values.detach().clone()
        return values.reshape(original_shape)


def make_timed_predict_action(monitor: LatencyMonitor, action_filter: ActionFilter | None = None):
    def timed_predict_action(
        observation: dict[str, Any],
        policy: PreTrainedPolicy,
        device: torch.device,
        preprocessor: PolicyProcessorPipeline[dict[str, Any], dict[str, Any]],
        postprocessor: PolicyProcessorPipeline[PolicyAction, PolicyAction],
        use_amp: bool,
        task: str | None = None,
        robot_type: str | None = None,
    ):
        observation = copy(observation)
        raw_observation = copy(observation)
        queue = _diffusion_action_queue(policy)
        queued_actions_before = len(queue) if queue is not None else None
        is_full_forward = queued_actions_before in (None, 0)

        with (
            torch.inference_mode(),
            torch.autocast(device_type=device.type) if device.type == "cuda" and use_amp else nullcontext(),
        ):
            t0 = time.perf_counter()
            observation = prepare_observation_for_inference(observation, device, task, robot_type)
            t1 = time.perf_counter()
            observation = preprocessor(observation)
            t2 = time.perf_counter()
            action = policy.select_action(observation)
            if device.type == "cuda":
                torch.cuda.synchronize(device)
            t3 = time.perf_counter()
            action = postprocessor(action)
            if action_filter is not None:
                action = action_filter(action, raw_observation)
            if device.type == "cuda":
                torch.cuda.synchronize(device)
            t4 = time.perf_counter()

        queue = _diffusion_action_queue(policy)
        queued_actions_after = len(queue) if queue is not None else None
        monitor.record_policy(
            prepare_ms=(t1 - t0) * 1000.0,
            preprocess_ms=(t2 - t1) * 1000.0,
            policy_ms=(t3 - t2) * 1000.0,
            postprocess_ms=(t4 - t3) * 1000.0,
            total_ms=(t4 - t0) * 1000.0,
            is_full_forward=is_full_forward,
            queued_actions_after=queued_actions_after,
        )
        return action

    return timed_predict_action


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model-path", default=DEFAULT_MODEL_ID)
    parser.add_argument("--dataset-id", default=DEFAULT_DATASET_ID)
    parser.add_argument("--remote-ip", default=DEFAULT_REMOTE_IP)
    parser.add_argument("--num-episodes", type=int, default=NUM_EPISODES)
    parser.add_argument("--episode-time-sec", type=int, default=EPISODE_TIME_SEC)
    parser.add_argument("--reset-time-sec", type=int, default=RESET_TIME_SEC)
    parser.add_argument("--fps", type=int, default=FPS)
    parser.add_argument("--task-description", default=TASK_DESCRIPTION)
    parser.add_argument(
        "--n-action-steps",
        type=int,
        default=None,
        help="How many queued diffusion actions to execute before querying the model again.",
    )
    parser.add_argument(
        "--num-inference-steps",
        type=int,
        default=NUM_INFERENCE_STEPS,
        help="Override reverse diffusion steps at inference time. Lower is faster but usually less accurate.",
    )
    parser.add_argument(
        "--trt-encoder-engine",
        type=Path,
        default=None,
        help="Optional TensorRT engine for the diffusion observation encoder.",
    )
    parser.add_argument(
        "--trt-unet-engine",
        type=Path,
        default=None,
        help="Optional TensorRT engine for the diffusion UNet denoiser.",
    )
    parser.add_argument(
        "--latency-print-every",
        type=int,
        default=25,
        help="Print a latency summary every N policy calls. Set to 0 to only print the final summary.",
    )
    parser.add_argument(
        "--action-ema-alpha",
        type=float,
        default=1.0,
        help="EMA smoothing for policy actions. 1.0 disables smoothing; smaller values are smoother/slower.",
    )
    parser.add_argument(
        "--arm-action-scale",
        type=float,
        default=1.0,
        help="Scale arm joint target deltas around the current observed joint position.",
    )
    parser.add_argument(
        "--base-velocity-scale",
        type=float,
        default=1.0,
        help="Scale x/y/theta velocity commands.",
    )
    parser.add_argument(
        "--max-arm-delta",
        type=float,
        default=None,
        help="Optional per-frame max arm joint target change in physical action units.",
    )
    parser.add_argument(
        "--latency-csv",
        default=None,
        help="Optional path to save per-policy-call latency samples as CSV.",
    )
    parser.add_argument(
        "--no-display-data",
        action="store_true",
        help="Disable live data display during evaluation.",
    )
    parser.add_argument(
        "--no-push-to-hub",
        action="store_true",
        help="Keep the evaluation dataset local instead of pushing it to the Hugging Face Hub.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    if not 0.0 < args.action_ema_alpha <= 1.0:
        raise ValueError("--action-ema-alpha must be in (0, 1].")
    if not 0.0 < args.arm_action_scale <= 1.0:
        raise ValueError("--arm-action-scale must be in (0, 1].")
    if not 0.0 <= args.base_velocity_scale <= 1.0:
        raise ValueError("--base-velocity-scale must be in [0, 1].")
    if args.max_arm_delta is not None and args.max_arm_delta <= 0.0:
        raise ValueError("--max-arm-delta must be positive.")

    robot_config = LeKiwiClientConfig(remote_ip=args.remote_ip, id="my_awesome_kiwi")
    robot_config.cameras = {
        name: replace(camera, width=IMAGE_SIZE, height=IMAGE_SIZE)
        for name, camera in robot_config.cameras.items()
    }
    robot = ResizeCameraObservationWrapper(LeKiwiClient(robot_config), IMAGE_SIZE)

    policy = DiffusionPolicy.from_pretrained(args.model_path, local_files_only=True)
    print(f"Loaded diffusion policy from: {args.model_path}")

    if args.n_action_steps is not None:
        if args.n_action_steps <= 0:
            raise ValueError("--n-action-steps must be positive.")
        max_action_steps = policy.config.horizon - policy.config.n_obs_steps + 1
        policy.config.n_action_steps = min(args.n_action_steps, max_action_steps)
        policy.reset()
        print(f"Using n_action_steps={policy.config.n_action_steps} for evaluation.")
    else:
        print(f"Using n_action_steps from checkpoint: {policy.config.n_action_steps}")

    if args.num_inference_steps is not None:
        if args.num_inference_steps <= 0:
            raise ValueError("--num-inference-steps must be positive.")
        policy.config.num_inference_steps = args.num_inference_steps
        policy.diffusion.num_inference_steps = args.num_inference_steps
        print(f"Using num_inference_steps={args.num_inference_steps} for evaluation.")
    else:
        print(f"Using num_inference_steps from checkpoint/model: {policy.diffusion.num_inference_steps}")

    if (args.trt_encoder_engine is None) != (args.trt_unet_engine is None):
        raise ValueError("--trt-encoder-engine and --trt-unet-engine must be provided together.")
    if args.trt_encoder_engine is not None and args.trt_unet_engine is not None:
        policy = attach_trt_diffusion_backend(
            policy,
            encoder_engine=args.trt_encoder_engine,
            unet_engine=args.trt_unet_engine,
            device="cuda",
        )
        policy.reset()
        print("Using TensorRT diffusion backend:")
        print(f"  encoder: {args.trt_encoder_engine}")
        print(f"  unet   : {args.trt_unet_engine}")

    action_features = hw_to_dataset_features(robot.action_features, ACTION)
    obs_features = hw_to_dataset_features(robot.observation_features, OBS_STR)
    dataset_features = {**action_features, **obs_features}

    dataset = LeRobotDataset.create(
        repo_id=args.dataset_id,
        fps=args.fps,
        features=dataset_features,
        robot_type=robot.name,
        use_videos=True,
        image_writer_threads=4,
    )

    preprocessor, postprocessor = make_pre_post_processors(
        policy_cfg=policy.config,
        pretrained_path=args.model_path,
        dataset_stats=dataset.meta.stats,
        preprocessor_overrides={"device_processor": {"device": str(policy.config.device)}},
        postprocessor_overrides={"device_processor": {"device": "cpu"}},
    )

    robot.connect()

    teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()
    listener, events = init_keyboard_listener()
    init_rerun(session_name="lekiwi_diffusion_evaluate")

    if not robot.is_connected:
        raise ValueError("Robot is not connected!")

    latency_monitor = LatencyMonitor(
        fps=args.fps,
        print_every=args.latency_print_every,
        csv_path=args.latency_csv,
    )
    action_filter = ActionFilter(
        action_names=list(dataset.features[ACTION]["names"]),
        state_names=list(dataset.features[OBS_STATE]["names"]),
        ema_alpha=args.action_ema_alpha,
        arm_action_scale=args.arm_action_scale,
        base_velocity_scale=args.base_velocity_scale,
        max_arm_delta=args.max_arm_delta,
    )
    if action_filter.enabled:
        print("Using action filter:")
        print(f"  action_ema_alpha   : {args.action_ema_alpha}")
        print(f"  arm_action_scale   : {args.arm_action_scale}")
        print(f"  base_velocity_scale: {args.base_velocity_scale}")
        print(f"  max_arm_delta      : {args.max_arm_delta}")

    timed_robot = TimedRobot(robot, latency_monitor)
    original_predict_action = lerobot_record.predict_action
    lerobot_record.predict_action = make_timed_predict_action(latency_monitor, action_filter)

    print("Starting diffusion evaluate loop...")
    try:
        recorded_episodes = 0
        while recorded_episodes < args.num_episodes and not events["stop_recording"]:
            log_say(
                f"Running diffusion inference, recording eval episode "
                f"{recorded_episodes} of {args.num_episodes}"
            )

            action_filter.reset()
            lerobot_record.record_loop(
                robot=timed_robot,
                events=events,
                fps=args.fps,
                policy=policy,
                preprocessor=preprocessor,
                postprocessor=postprocessor,
                dataset=dataset,
                control_time_s=args.episode_time_sec,
                single_task=args.task_description,
                display_data=not args.no_display_data,
                teleop_action_processor=teleop_action_processor,
                robot_action_processor=robot_action_processor,
                robot_observation_processor=robot_observation_processor,
            )

            if not events["stop_recording"] and (
                (recorded_episodes < args.num_episodes - 1) or events["rerecord_episode"]
            ):
                log_say("Reset the environment")
                lerobot_record.record_loop(
                    robot=timed_robot,
                    events=events,
                    fps=args.fps,
                    control_time_s=args.reset_time_sec,
                    single_task=args.task_description,
                    display_data=not args.no_display_data,
                    teleop_action_processor=teleop_action_processor,
                    robot_action_processor=robot_action_processor,
                    robot_observation_processor=robot_observation_processor,
                )

            if events["rerecord_episode"]:
                log_say("Re-record episode")
                events["rerecord_episode"] = False
                events["exit_early"] = False
                dataset.clear_episode_buffer()
                continue

            dataset.save_episode()
            recorded_episodes += 1
    finally:
        lerobot_record.predict_action = original_predict_action
        latency_monitor.print_final_summary()
        latency_monitor.save_csv()

        log_say("Stop recording")
        robot.disconnect()
        if listener is not None:
            listener.stop()

        dataset.finalize()
        if not args.no_push_to_hub:
            dataset.push_to_hub()


if __name__ == "__main__":
    main()
