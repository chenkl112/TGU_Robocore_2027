#!/usr/bin/env python3
"""Convert the checked-in static YOLOv8-pose IR from 416x416 to 640x480."""

import argparse
from pathlib import Path

import numpy as np
import openvino as ov
from openvino import opset13 as ops


NETWORK_WIDTH = 640
NETWORK_HEIGHT = 480
STRIDES = (8, 16, 32)


def detection_head_constants(height: int, width: int) -> dict[str, np.ndarray]:
    anchors = []
    strides = []
    origins = []
    for stride in STRIDES:
        grid_height = height // stride
        grid_width = width // stride
        yy, xx = np.meshgrid(
            np.arange(grid_height, dtype=np.float32),
            np.arange(grid_width, dtype=np.float32),
            indexing="ij",
        )
        anchors.append(np.stack((xx + 0.5, yy + 0.5)).reshape(2, -1))
        origins.append(np.stack((xx * stride, yy * stride)).reshape(2, -1))
        strides.append(np.full((1, grid_height * grid_width), stride, np.float32))

    anchor_points = np.concatenate(anchors, axis=1)[None, ...]
    stride_tensor = np.concatenate(strides, axis=1)[None, ...]
    keypoint_origins = np.concatenate(origins, axis=1)[None, None, ...]
    return {
        "/model.28/dfl/Constant_output_0": np.array([1, 4, 16, -1], np.int64),
        "/model.28/dfl/Constant_1_output_0": np.array([1, 4, -1], np.int64),
        "/model.28/Constant_13_output_0": anchor_points,
        "Constant_7137": stride_tensor,
        "Constant_7138": (stride_tensor * 2)[:, None, ...],
        "Constant_7139": keypoint_origins,
    }


def convert(source: Path, output: Path) -> None:
    core = ov.Core()
    model = core.read_model(source)
    if model.input().shape != ov.Shape([1, 3, 416, 416]):
        raise RuntimeError(f"expected source shape 1x3x416x416, got {model.input().shape}")

    model.input().get_node().set_partial_shape(
        ov.PartialShape([1, 3, NETWORK_HEIGHT, NETWORK_WIDTH])
    )
    replacements = detection_head_constants(NETWORK_HEIGHT, NETWORK_WIDTH)
    replaced = set()
    for node in list(model.get_ordered_ops()):
        name = node.get_friendly_name()
        if name not in replacements:
            continue
        constant = ops.constant(replacements[name])
        constant.set_friendly_name(name)
        node.output(0).replace(constant.output(0))
        replaced.add(name)

    missing = replacements.keys() - replaced
    if missing:
        raise RuntimeError(f"model layout changed; missing nodes: {sorted(missing)}")

    model.validate_nodes_and_infer_types()
    expected_output = ov.Shape([1, 14, 6300])
    if model.output().shape != expected_output:
        raise RuntimeError(f"unexpected converted output shape: {model.output().shape}")
    model.set_rt_info("[480, 640]", ["framework", "imgsz"])

    output.parent.mkdir(parents=True, exist_ok=True)
    ov.serialize(model, output, output.with_suffix(".bin"))

    compiled = core.compile_model(model, "CPU")
    result = compiled(np.zeros((1, 3, NETWORK_HEIGHT, NETWORK_WIDTH), np.float32))[0]
    if result.shape != tuple(expected_output) or not np.isfinite(result).all():
        raise RuntimeError("converted model failed the zero-input inference check")

    print(f"generated {output} and {output.with_suffix('.bin')}")
    print(f"input: 1x3x{NETWORK_HEIGHT}x{NETWORK_WIDTH}, output: 1x14x6300")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--source", type=Path, default=Path("assets/yolov8_416x416.xml")
    )
    parser.add_argument(
        "--output", type=Path, default=Path("assets/yolov8_640x480.xml")
    )
    args = parser.parse_args()
    convert(args.source, args.output)


if __name__ == "__main__":
    main()
