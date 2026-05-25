#!/usr/bin/env python

"""Build a TensorRT engine from an ONNX model using the Python TensorRT API."""

from __future__ import annotations

import argparse
import re
from pathlib import Path

import tensorrt as trt


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Build a TensorRT .engine file from ONNX.")
    parser.add_argument("--onnx", type=Path, required=True, help="Input ONNX model path.")
    parser.add_argument("--engine", type=Path, required=True, help="Output TensorRT engine path.")
    parser.add_argument("--fp16", action="store_true", help="Enable FP16 kernels when available.")
    parser.add_argument(
        "--workspace-gb",
        type=float,
        default=2.0,
        help="TensorRT workspace memory pool limit in GiB.",
    )
    parser.add_argument(
        "--min-shapes",
        nargs="*",
        default=None,
        metavar="NAME:SHAPE",
        help="Dynamic profile min shapes, for example sample:1x16x9 timestep:1 global_cond:1x274.",
    )
    parser.add_argument(
        "--opt-shapes",
        nargs="*",
        default=None,
        metavar="NAME:SHAPE",
        help="Dynamic profile opt shapes.",
    )
    parser.add_argument(
        "--max-shapes",
        nargs="*",
        default=None,
        metavar="NAME:SHAPE",
        help="Dynamic profile max shapes.",
    )
    return parser.parse_args()


def parse_shape(shape_text: str) -> tuple[int, ...]:
    parts = [part for part in re.split(r"[x,]", shape_text) if part]
    if not parts:
        raise ValueError(f"Invalid shape: {shape_text}")
    return tuple(int(part) for part in parts)


def parse_named_shapes(items: list[str] | None) -> dict[str, tuple[int, ...]]:
    if not items:
        return {}

    shapes = {}
    for item in items:
        if ":" in item:
            name, shape = item.split(":", 1)
        elif "=" in item:
            name, shape = item.split("=", 1)
        else:
            raise ValueError(f"Expected NAME:SHAPE, got: {item}")
        shapes[name] = parse_shape(shape)
    return shapes


def has_dynamic_input(network: trt.INetworkDefinition) -> bool:
    for i in range(network.num_inputs):
        if any(dim < 0 for dim in network.get_input(i).shape):
            return True
    return False


def print_network_io(network: trt.INetworkDefinition) -> None:
    print("Network inputs:")
    for i in range(network.num_inputs):
        tensor = network.get_input(i)
        print(f"  {tensor.name}: shape={tuple(tensor.shape)}, dtype={tensor.dtype}")

    print("Network outputs:")
    for i in range(network.num_outputs):
        tensor = network.get_output(i)
        print(f"  {tensor.name}: shape={tuple(tensor.shape)}, dtype={tensor.dtype}")


def add_optimization_profile(
    builder: trt.Builder,
    config: trt.IBuilderConfig,
    network: trt.INetworkDefinition,
    min_shapes: dict[str, tuple[int, ...]],
    opt_shapes: dict[str, tuple[int, ...]],
    max_shapes: dict[str, tuple[int, ...]],
) -> None:
    profile = builder.create_optimization_profile()

    for i in range(network.num_inputs):
        tensor = network.get_input(i)
        shape = tuple(tensor.shape)
        if not any(dim < 0 for dim in shape):
            continue

        name = tensor.name
        if name not in min_shapes or name not in opt_shapes or name not in max_shapes:
            raise ValueError(
                "Dynamic ONNX input requires --min-shapes/--opt-shapes/--max-shapes for "
                f"'{name}'. Network shape is {shape}."
            )
        profile.set_shape(name, min_shapes[name], opt_shapes[name], max_shapes[name])
        print(
            f"Profile {name}: min={min_shapes[name]}, opt={opt_shapes[name]}, max={max_shapes[name]}"
        )

    config.add_optimization_profile(profile)


def main() -> None:
    args = parse_args()
    if not args.onnx.is_file():
        raise FileNotFoundError(f"ONNX file does not exist: {args.onnx}")

    logger = trt.Logger(trt.Logger.INFO)
    builder = trt.Builder(logger)
    flags = 1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)
    network = builder.create_network(flags)
    parser = trt.OnnxParser(network, logger)

    print(f"TensorRT version: {trt.__version__}")
    print(f"Parsing ONNX: {args.onnx}")
    if not parser.parse(args.onnx.read_bytes()):
        for i in range(parser.num_errors):
            print(parser.get_error(i))
        raise RuntimeError("Failed to parse ONNX model.")

    print_network_io(network)

    config = builder.create_builder_config()
    workspace_bytes = int(args.workspace_gb * (1024**3))
    config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, workspace_bytes)

    if args.fp16:
        if not builder.platform_has_fast_fp16:
            print("Warning: platform_has_fast_fp16 is false, but FP16 flag will still be requested.")
        config.set_flag(trt.BuilderFlag.FP16)

    if has_dynamic_input(network):
        add_optimization_profile(
            builder,
            config,
            network,
            parse_named_shapes(args.min_shapes),
            parse_named_shapes(args.opt_shapes),
            parse_named_shapes(args.max_shapes),
        )

    print(f"Building engine: {args.engine}")
    serialized_engine = builder.build_serialized_network(network, config)
    if serialized_engine is None:
        raise RuntimeError("TensorRT engine build failed.")

    args.engine.parent.mkdir(parents=True, exist_ok=True)
    args.engine.write_bytes(bytes(serialized_engine))
    print(f"Wrote engine: {args.engine}")


if __name__ == "__main__":
    main()
