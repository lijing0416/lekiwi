#!/usr/bin/env python

"""Export the LeKiwi diffusion policy pieces to ONNX for TensorRT.

The complete diffusion rollout contains a Python scheduler loop. For TensorRT it
is usually better to export the neural pieces and keep the scheduler in runtime
code:

1. encoder: normalized observations -> global conditioning vector
2. unet: noisy action sample + timestep + global conditioning -> denoising output
3. denoise_step: normalized observations + noisy action sample + timestep -> denoising output
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

import torch
from torch import Tensor, nn

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from lerobot.policies.diffusion.modeling_diffusion import DiffusionPolicy  # noqa: E402
from lerobot.utils.constants import OBS_ENV_STATE, OBS_IMAGES, OBS_STATE  # noqa: E402


DEFAULT_CHECKPOINT = (
    REPO_ROOT
    / "outputs/train/train/diffusion_aloha_merged_20260517_11datasets_bs32_aug"
    / "checkpoints/030000/pretrained_model"
)
DEFAULT_OUTPUT_DIR = REPO_ROOT / "outputs/onnx/diffusion_aloha_merged_030000"


def onnx_name(name: str) -> str:
    return re.sub(r"[^0-9a-zA-Z_]+", "_", name).strip("_")


class DiffusionConditioningEncoder(nn.Module):
    """Wrap `_prepare_global_conditioning` with positional tensor inputs."""

    def __init__(self, diffusion: nn.Module, image_keys: list[str], has_env_state: bool):
        super().__init__()
        self.diffusion = diffusion
        self.image_keys = image_keys
        self.has_env_state = has_env_state

    def forward(self, observation_state: Tensor, *inputs: Tensor) -> Tensor:
        batch = {OBS_STATE: observation_state}

        image_count = len(self.image_keys)
        if image_count:
            image_inputs = inputs[:image_count]
            batch[OBS_IMAGES] = torch.stack(tuple(image_inputs), dim=2)

        if self.has_env_state:
            batch[OBS_ENV_STATE] = inputs[image_count]

        return self.diffusion._prepare_global_conditioning(batch)


class DiffusionUnetDenoiser(nn.Module):
    """Wrap the neural denoiser used at every reverse diffusion timestep."""

    def __init__(self, unet: nn.Module):
        super().__init__()
        self.unet = unet

    def forward(self, sample: Tensor, timestep: Tensor, global_cond: Tensor) -> Tensor:
        return self.unet(sample, timestep, global_cond=global_cond)


class DiffusionDenoiseStep(nn.Module):
    """Convenience graph that recomputes conditioning and runs one denoise step."""

    def __init__(self, diffusion: nn.Module, image_keys: list[str], has_env_state: bool):
        super().__init__()
        self.diffusion = diffusion
        self.image_keys = image_keys
        self.has_env_state = has_env_state

    def forward(self, observation_state: Tensor, *inputs: Tensor) -> Tensor:
        sample = inputs[-2]
        timestep = inputs[-1]
        cond_inputs = inputs[:-2]

        batch = {OBS_STATE: observation_state}
        image_count = len(self.image_keys)
        if image_count:
            batch[OBS_IMAGES] = torch.stack(tuple(cond_inputs[:image_count]), dim=2)
        if self.has_env_state:
            batch[OBS_ENV_STATE] = cond_inputs[image_count]

        global_cond = self.diffusion._prepare_global_conditioning(batch)
        return self.diffusion.unet(sample, timestep, global_cond=global_cond)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Export LeKiwi DiffusionPolicy modules to ONNX.")
    parser.add_argument(
        "--checkpoint",
        type=Path,
        default=DEFAULT_CHECKPOINT,
        help="Path to a DiffusionPolicy pretrained_model checkpoint directory.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help="Directory where ONNX files and metadata will be written.",
    )
    parser.add_argument(
        "--device",
        default="cuda",
        choices=["cuda", "cpu"],
        help="Device used while tracing the model.",
    )
    parser.add_argument("--batch-size", type=int, default=1, help="Dummy batch size used for export.")
    parser.add_argument("--opset", type=int, default=17, help="ONNX opset version.")
    parser.add_argument(
        "--dynamic-batch",
        action="store_true",
        help="Mark batch dimension as dynamic. TensorRT then needs an optimization profile.",
    )
    parser.add_argument(
        "--num-inference-steps",
        type=int,
        default=None,
        help="Optional value recorded in metadata for the runtime scheduler.",
    )
    parser.add_argument(
        "--mode",
        choices=["all", "encoder", "unet", "denoise_step"],
        default="all",
        help="Which ONNX graph to export.",
    )
    parser.add_argument("--skip-check", action="store_true", help="Skip onnx.checker validation.")
    return parser.parse_args()


def check_args(args: argparse.Namespace) -> None:
    if not args.checkpoint.is_dir():
        raise FileNotFoundError(f"Checkpoint directory does not exist: {args.checkpoint}")
    if args.batch_size <= 0:
        raise ValueError("--batch-size must be positive.")
    if args.num_inference_steps is not None and args.num_inference_steps <= 0:
        raise ValueError("--num-inference-steps must be positive.")
    if args.device == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA is not available. Re-run with --device cpu.")


def load_policy(checkpoint: Path, device: torch.device, num_inference_steps: int | None) -> DiffusionPolicy:
    policy = DiffusionPolicy.from_pretrained(checkpoint, local_files_only=True)
    policy.to(device)
    policy.eval()
    policy.config.device = str(device)

    if num_inference_steps is not None:
        policy.config.num_inference_steps = num_inference_steps
        policy.diffusion.num_inference_steps = num_inference_steps

    return policy


def make_dummy_observation_inputs(policy: DiffusionPolicy, batch_size: int, device: torch.device) -> tuple:
    config = policy.config
    dtype = next(policy.parameters()).dtype

    obs_inputs: list[Tensor] = [
        torch.randn(batch_size, config.n_obs_steps, *config.robot_state_feature.shape, device=device, dtype=dtype)
    ]

    for feature in config.image_features.values():
        obs_inputs.append(
            torch.randn(batch_size, config.n_obs_steps, *feature.shape, device=device, dtype=dtype)
        )

    if config.env_state_feature:
        obs_inputs.append(
            torch.randn(batch_size, config.n_obs_steps, *config.env_state_feature.shape, device=device, dtype=dtype)
        )

    return tuple(obs_inputs)


def make_dummy_unet_inputs(policy: DiffusionPolicy, batch_size: int, device: torch.device) -> tuple:
    config = policy.config
    dtype = next(policy.parameters()).dtype

    with torch.no_grad():
        encoder = DiffusionConditioningEncoder(
            policy.diffusion, list(config.image_features), config.env_state_feature is not None
        ).to(device)
        global_cond = encoder(*make_dummy_observation_inputs(policy, batch_size, device))

    sample = torch.randn(
        batch_size,
        config.horizon,
        config.action_feature.shape[0],
        device=device,
        dtype=dtype,
    )
    timestep = torch.full((batch_size,), config.num_train_timesteps - 1, device=device, dtype=torch.long)
    return sample, timestep, global_cond


def observation_input_names(policy: DiffusionPolicy) -> list[str]:
    names = [onnx_name(OBS_STATE)]
    names.extend(onnx_name(key) for key in policy.config.image_features)
    if policy.config.env_state_feature:
        names.append(onnx_name(OBS_ENV_STATE))
    return names


def global_cond_dim(policy: DiffusionPolicy) -> int:
    config = policy.config
    dim_per_step = config.robot_state_feature.shape[0]

    if config.image_features:
        if config.use_separate_rgb_encoder_per_camera:
            image_feature_dim = policy.diffusion.rgb_encoder[0].feature_dim
        else:
            image_feature_dim = policy.diffusion.rgb_encoder.feature_dim
        dim_per_step += image_feature_dim * len(config.image_features)

    if config.env_state_feature:
        dim_per_step += config.env_state_feature.shape[0]

    return dim_per_step * config.n_obs_steps


def batch_dynamic_axes(input_names: list[str], output_names: list[str], enabled: bool) -> dict | None:
    if not enabled:
        return None
    return {name: {0: "batch"} for name in [*input_names, *output_names]}


def export_onnx(
    model: nn.Module,
    dummy_inputs: tuple[Tensor, ...],
    output_path: Path,
    input_names: list[str],
    output_names: list[str],
    opset: int,
    dynamic_batch: bool,
    skip_check: bool,
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with torch.no_grad():
        torch.onnx.export(
            model,
            dummy_inputs,
            str(output_path),
            export_params=True,
            do_constant_folding=True,
            input_names=input_names,
            output_names=output_names,
            opset_version=opset,
            dynamic_axes=batch_dynamic_axes(input_names, output_names, dynamic_batch),
        )

    if not skip_check:
        import onnx

        onnx_model = onnx.load(output_path)
        onnx.checker.check_model(onnx_model)

    print(f"Exported: {output_path}")


def write_metadata(policy: DiffusionPolicy, args: argparse.Namespace, files: dict[str, str]) -> None:
    config = policy.config
    metadata = {
        "checkpoint": str(args.checkpoint),
        "opset": args.opset,
        "dynamic_batch": args.dynamic_batch,
        "files": files,
        "input_note": "ONNX inputs should already be processed by policy_preprocessor/normalizer.",
        "n_obs_steps": config.n_obs_steps,
        "horizon": config.horizon,
        "n_action_steps": config.n_action_steps,
        "action_dim": config.action_feature.shape[0],
        "num_train_timesteps": config.num_train_timesteps,
        "num_inference_steps": policy.diffusion.num_inference_steps,
        "observation_inputs": {
            onnx_name(OBS_STATE): [args.batch_size, config.n_obs_steps, *config.robot_state_feature.shape],
            **{
                onnx_name(key): [args.batch_size, config.n_obs_steps, *feature.shape]
                for key, feature in config.image_features.items()
            },
        },
        "unet_inputs": {
            "sample": [args.batch_size, config.horizon, config.action_feature.shape[0]],
            "timestep": [args.batch_size],
            "global_cond": [args.batch_size, global_cond_dim(policy)],
        },
    }
    if config.env_state_feature:
        metadata["observation_inputs"][onnx_name(OBS_ENV_STATE)] = [
            args.batch_size,
            config.n_obs_steps,
            *config.env_state_feature.shape,
        ]

    metadata_path = args.output_dir / "metadata.json"
    metadata_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    print(f"Wrote metadata: {metadata_path}")


def main() -> None:
    args = parse_args()
    check_args(args)

    device = torch.device(args.device)
    policy = load_policy(args.checkpoint, device, args.num_inference_steps)
    config = policy.config

    image_keys = list(config.image_features)
    has_env_state = config.env_state_feature is not None
    obs_names = observation_input_names(policy)
    files: dict[str, str] = {}

    if args.mode in {"all", "encoder"}:
        encoder = DiffusionConditioningEncoder(policy.diffusion, image_keys, has_env_state).to(device).eval()
        output_path = args.output_dir / "diffusion_encoder.onnx"
        export_onnx(
            encoder,
            make_dummy_observation_inputs(policy, args.batch_size, device),
            output_path,
            obs_names,
            ["global_cond"],
            args.opset,
            args.dynamic_batch,
            args.skip_check,
        )
        files["encoder"] = str(output_path)

    if args.mode in {"all", "unet"}:
        unet = DiffusionUnetDenoiser(policy.diffusion.unet).to(device).eval()
        output_path = args.output_dir / "diffusion_unet.onnx"
        export_onnx(
            unet,
            make_dummy_unet_inputs(policy, args.batch_size, device),
            output_path,
            ["sample", "timestep", "global_cond"],
            ["model_output"],
            args.opset,
            args.dynamic_batch,
            args.skip_check,
        )
        files["unet"] = str(output_path)

    if args.mode in {"all", "denoise_step"}:
        denoise_step = DiffusionDenoiseStep(policy.diffusion, image_keys, has_env_state).to(device).eval()
        sample, timestep, _ = make_dummy_unet_inputs(policy, args.batch_size, device)
        output_path = args.output_dir / "diffusion_denoise_step.onnx"
        export_onnx(
            denoise_step,
            (*make_dummy_observation_inputs(policy, args.batch_size, device), sample, timestep),
            output_path,
            [*obs_names, "sample", "timestep"],
            ["model_output"],
            args.opset,
            args.dynamic_batch,
            args.skip_check,
        )
        files["denoise_step"] = str(output_path)

    write_metadata(policy, args, files)


if __name__ == "__main__":
    main()
