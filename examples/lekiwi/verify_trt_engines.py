#!/usr/bin/env python

"""Compare TensorRT diffusion engines with the PyTorch policy modules."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import tensorrt as trt
import torch

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from export_onnx import (  # noqa: E402
    DEFAULT_CHECKPOINT,
    DiffusionConditioningEncoder,
    DiffusionUnetDenoiser,
    make_dummy_observation_inputs,
    make_dummy_unet_inputs,
    observation_input_names,
)
from lerobot.policies.diffusion.modeling_diffusion import DiffusionPolicy  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Validate TensorRT engines against PyTorch outputs.")
    parser.add_argument("--checkpoint", type=Path, default=DEFAULT_CHECKPOINT)
    parser.add_argument("--encoder-engine", type=Path, default=None)
    parser.add_argument("--unet-engine", type=Path, default=None)
    parser.add_argument("--batch-size", type=int, default=1)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--warmup", type=int, default=10)
    parser.add_argument("--repeat", type=int, default=50)
    return parser.parse_args()


def trt_dtype_to_torch(dtype: trt.DataType) -> torch.dtype:
    mapping = {
        trt.DataType.FLOAT: torch.float32,
        trt.DataType.HALF: torch.float16,
        trt.DataType.INT8: torch.int8,
        trt.DataType.INT32: torch.int32,
        trt.DataType.INT64: torch.int64,
        trt.DataType.BOOL: torch.bool,
    }
    if dtype not in mapping:
        raise TypeError(f"Unsupported TensorRT dtype: {dtype}")
    return mapping[dtype]


class TensorRTRunner:
    def __init__(self, engine_path: Path):
        if not engine_path.is_file():
            raise FileNotFoundError(f"TensorRT engine does not exist: {engine_path}")

        logger = trt.Logger(trt.Logger.WARNING)
        runtime = trt.Runtime(logger)
        engine = runtime.deserialize_cuda_engine(engine_path.read_bytes())
        if engine is None:
            raise RuntimeError(f"Failed to deserialize TensorRT engine: {engine_path}")

        self.engine_path = engine_path
        self.runtime = runtime
        self.engine = engine
        self.context = engine.create_execution_context()
        self.input_names: list[str] = []
        self.output_names: list[str] = []

        for i in range(engine.num_io_tensors):
            name = engine.get_tensor_name(i)
            mode = engine.get_tensor_mode(name)
            if mode == trt.TensorIOMode.INPUT:
                self.input_names.append(name)
            else:
                self.output_names.append(name)

        print(f"Loaded engine: {engine_path}")
        print(f"  inputs : {self.input_names}")
        print(f"  outputs: {self.output_names}")

    def run(self, inputs: dict[str, torch.Tensor], sync: bool = True) -> dict[str, torch.Tensor]:
        prepared_inputs = {}

        for name in self.input_names:
            if name not in inputs:
                raise KeyError(f"Missing input '{name}' for engine {self.engine_path}")

            dtype = trt_dtype_to_torch(self.engine.get_tensor_dtype(name))
            tensor = inputs[name].to(device="cuda", dtype=dtype).contiguous()

            engine_shape = tuple(self.engine.get_tensor_shape(name))
            if any(dim < 0 for dim in engine_shape):
                self.context.set_input_shape(name, tuple(tensor.shape))

            self.context.set_tensor_address(name, tensor.data_ptr())
            prepared_inputs[name] = tensor

        outputs = {}
        for name in self.output_names:
            shape = tuple(self.context.get_tensor_shape(name))
            dtype = trt_dtype_to_torch(self.engine.get_tensor_dtype(name))
            outputs[name] = torch.empty(shape, device="cuda", dtype=dtype)
            self.context.set_tensor_address(name, outputs[name].data_ptr())

        ok = self.context.execute_async_v3(torch.cuda.current_stream().cuda_stream)
        if not ok:
            raise RuntimeError(f"TensorRT execution failed for {self.engine_path}")
        if sync:
            torch.cuda.synchronize()

        return outputs


def report_diff(name: str, reference: torch.Tensor, actual: torch.Tensor) -> None:
    reference = reference.float()
    actual = actual.float()
    diff = (reference - actual).abs()
    max_abs = diff.max().item()
    mean_abs = diff.mean().item()
    ref_scale = reference.abs().max().clamp_min(1e-6)
    max_rel = (diff.max() / ref_scale).item()
    print(f"{name}: max_abs={max_abs:.6g}, mean_abs={mean_abs:.6g}, max_rel={max_rel:.6g}")


def benchmark(name: str, runner: TensorRTRunner, inputs: dict[str, torch.Tensor], warmup: int, repeat: int) -> None:
    for _ in range(warmup):
        runner.run(inputs, sync=False)
    torch.cuda.synchronize()

    start = time.perf_counter()
    for _ in range(repeat):
        runner.run(inputs, sync=False)
    torch.cuda.synchronize()
    elapsed_ms = (time.perf_counter() - start) * 1000.0 / repeat
    print(f"{name}: avg_latency={elapsed_ms:.3f} ms over {repeat} runs")


def main() -> None:
    args = parse_args()
    if args.encoder_engine is None and args.unet_engine is None:
        raise ValueError("Provide --encoder-engine, --unet-engine, or both.")
    if not torch.cuda.is_available():
        raise RuntimeError("CUDA is required for TensorRT engine verification.")

    torch.manual_seed(args.seed)
    torch.cuda.manual_seed_all(args.seed)

    policy = DiffusionPolicy.from_pretrained(args.checkpoint, local_files_only=True)
    policy.to("cuda").eval()
    policy.config.device = "cuda"
    image_keys = list(policy.config.image_features)
    has_env_state = policy.config.env_state_feature is not None

    if args.encoder_engine is not None:
        encoder_runner = TensorRTRunner(args.encoder_engine)
        obs_inputs = make_dummy_observation_inputs(policy, args.batch_size, torch.device("cuda"))
        obs_names = observation_input_names(policy)
        encoder_inputs = dict(zip(obs_names, obs_inputs, strict=True))

        encoder = DiffusionConditioningEncoder(policy.diffusion, image_keys, has_env_state).to("cuda").eval()
        with torch.inference_mode():
            reference = encoder(*obs_inputs)
        actual = encoder_runner.run(encoder_inputs)["global_cond"]
        report_diff("encoder.global_cond", reference, actual)
        benchmark("encoder", encoder_runner, encoder_inputs, args.warmup, args.repeat)

    if args.unet_engine is not None:
        unet_runner = TensorRTRunner(args.unet_engine)
        sample, timestep, global_cond = make_dummy_unet_inputs(
            policy, args.batch_size, torch.device("cuda")
        )
        unet_inputs = {
            "sample": sample,
            "timestep": timestep,
            "global_cond": global_cond,
        }

        unet = DiffusionUnetDenoiser(policy.diffusion.unet).to("cuda").eval()
        with torch.inference_mode():
            reference = unet(sample, timestep, global_cond)
        actual = unet_runner.run(unet_inputs)["model_output"]
        report_diff("unet.model_output", reference, actual)
        benchmark("unet", unet_runner, unet_inputs, args.warmup, args.repeat)


if __name__ == "__main__":
    main()
