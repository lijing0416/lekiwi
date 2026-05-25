#!/usr/bin/env python

"""TensorRT backend for the LeRobot DiffusionPolicy inference path."""

from __future__ import annotations

import gc
import re
from pathlib import Path
from typing import Any

import torch
from torch import Tensor, nn

from lerobot.utils.constants import OBS_ENV_STATE, OBS_IMAGES, OBS_STATE


def onnx_name(name: str) -> str:
    return re.sub(r"[^0-9a-zA-Z_]+", "_", name).strip("_")


def _import_tensorrt():
    try:
        import tensorrt as trt
    except ImportError as exc:
        raise ImportError(
            "TensorRT Python package is required for --trt-encoder-engine/--trt-unet-engine."
        ) from exc
    return trt


def _trt_dtype_to_torch(dtype: Any) -> torch.dtype:
    trt = _import_tensorrt()
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


class TensorRTEngineRunner:
    """Small synchronous TensorRT runner backed by torch CUDA tensors."""

    def __init__(self, engine_path: str | Path, device: str = "cuda"):
        if device != "cuda":
            raise ValueError("TensorRT engine execution requires device='cuda'.")
        if not torch.cuda.is_available():
            raise RuntimeError("CUDA is required for TensorRT engine execution.")

        trt = _import_tensorrt()
        engine_path = Path(engine_path).expanduser()
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
        self.stream = torch.cuda.Stream()
        self.input_names: list[str] = []
        self.output_names: list[str] = []

        for i in range(engine.num_io_tensors):
            name = engine.get_tensor_name(i)
            if engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT:
                self.input_names.append(name)
            else:
                self.output_names.append(name)

    def run(self, inputs: dict[str, Tensor]) -> dict[str, Tensor]:
        current_stream = torch.cuda.current_stream()
        self.stream.wait_stream(current_stream)

        with torch.cuda.stream(self.stream):
            prepared_inputs = {}
            for name in self.input_names:
                if name not in inputs:
                    raise KeyError(f"Missing TensorRT input '{name}' for {self.engine_path}")

                dtype = _trt_dtype_to_torch(self.engine.get_tensor_dtype(name))
                tensor = inputs[name].to(device="cuda", dtype=dtype).contiguous()

                engine_shape = tuple(self.engine.get_tensor_shape(name))
                if any(dim < 0 for dim in engine_shape):
                    self.context.set_input_shape(name, tuple(tensor.shape))
                elif tuple(tensor.shape) != engine_shape:
                    raise ValueError(
                        f"Input '{name}' for {self.engine_path} has shape {tuple(tensor.shape)}, "
                        f"but the engine expects {engine_shape}."
                    )

                self.context.set_tensor_address(name, tensor.data_ptr())
                tensor.record_stream(self.stream)
                prepared_inputs[name] = tensor

            outputs = {}
            for name in self.output_names:
                shape = tuple(self.context.get_tensor_shape(name))
                dtype = _trt_dtype_to_torch(self.engine.get_tensor_dtype(name))
                outputs[name] = torch.empty(shape, device="cuda", dtype=dtype)
                self.context.set_tensor_address(name, outputs[name].data_ptr())

            ok = self.context.execute_async_v3(self.stream.cuda_stream)
            if not ok:
                raise RuntimeError(f"TensorRT execution failed for {self.engine_path}")

        current_stream.wait_stream(self.stream)
        return outputs


class TensorRTDiffusionModel(nn.Module):
    """Drop-in inference backend for `DiffusionPolicy.diffusion`."""

    def __init__(
        self,
        torch_diffusion: nn.Module,
        encoder_engine: str | Path,
        unet_engine: str | Path,
        device: str = "cuda",
    ):
        super().__init__()
        self.config = torch_diffusion.config
        self.noise_scheduler = torch_diffusion.noise_scheduler
        self.num_inference_steps = torch_diffusion.num_inference_steps
        self.device = torch.device(device)
        self.image_keys = list(self.config.image_features)
        self.has_env_state = self.config.env_state_feature is not None
        self.encoder = TensorRTEngineRunner(encoder_engine, device=device)
        self.unet = TensorRTEngineRunner(unet_engine, device=device)

    def _encoder_inputs(self, batch: dict[str, Tensor]) -> dict[str, Tensor]:
        inputs = {onnx_name(OBS_STATE): batch[OBS_STATE]}

        if self.image_keys:
            images = batch[OBS_IMAGES]
            for camera_idx, key in enumerate(self.image_keys):
                inputs[onnx_name(key)] = images[:, :, camera_idx]

        if self.has_env_state:
            inputs[onnx_name(OBS_ENV_STATE)] = batch[OBS_ENV_STATE]

        return inputs

    def _prepare_global_conditioning(self, batch: dict[str, Tensor]) -> Tensor:
        outputs = self.encoder.run(self._encoder_inputs(batch))
        return outputs["global_cond"]

    def conditional_sample(
        self,
        batch_size: int,
        global_cond: Tensor | None = None,
        generator: torch.Generator | None = None,
        noise: Tensor | None = None,
    ) -> Tensor:
        if global_cond is None:
            raise ValueError("TensorRT diffusion backend requires global_cond.")

        sample = (
            noise.to(device=self.device, dtype=torch.float32).contiguous()
            if noise is not None
            else torch.randn(
                size=(batch_size, self.config.horizon, self.config.action_feature.shape[0]),
                dtype=torch.float32,
                device=self.device,
                generator=generator,
            )
        )
        global_cond = global_cond.to(device=self.device, dtype=torch.float32).contiguous()

        self.noise_scheduler.set_timesteps(self.num_inference_steps)

        for t in self.noise_scheduler.timesteps:
            timestep_value = int(t.item()) if isinstance(t, Tensor) else int(t)
            timestep = torch.full(
                sample.shape[:1],
                timestep_value,
                dtype=torch.long,
                device=self.device,
            )
            model_output = self.unet.run(
                {
                    "sample": sample,
                    "timestep": timestep,
                    "global_cond": global_cond,
                }
            )["model_output"]
            sample = self.noise_scheduler.step(model_output, t, sample, generator=generator).prev_sample

        return sample

    def generate_actions(self, batch: dict[str, Tensor], noise: Tensor | None = None) -> Tensor:
        batch_size = batch[OBS_STATE].shape[0]
        global_cond = self._prepare_global_conditioning(batch)
        return self.conditional_sample(batch_size, global_cond=global_cond, noise=noise)

    def compute_loss(self, batch: dict[str, Tensor]) -> Tensor:
        raise RuntimeError("TensorRTDiffusionModel is inference-only and cannot compute training loss.")


def attach_trt_diffusion_backend(
    policy: nn.Module,
    encoder_engine: str | Path,
    unet_engine: str | Path,
    device: str = "cuda",
) -> nn.Module:
    """Replace `policy.diffusion` with an inference-only TensorRT backend."""

    policy.diffusion = TensorRTDiffusionModel(
        policy.diffusion,
        encoder_engine=encoder_engine,
        unet_engine=unet_engine,
        device=device,
    )
    policy.config.device = device
    policy.to(device)
    policy.eval()
    gc.collect()
    torch.cuda.empty_cache()
    return policy
