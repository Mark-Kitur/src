#!/usr/bin/env python3
"""Raspberry Pi 5 waste detection using the user's exact YOLO26n ONNX export.

Verified export metadata (2026-10-07):
  images: float32 [1,3,640,640]
  output0: float32 [1,7,8400]
  0=metal, 1=organic, 2=plastic; no embedded NMS.

Usage (inside virtual environment):
  python agv_waste_yolo26.py --image ~/agv_project/camera_tests/test.jpg
  python agv_waste_yolo26.py --camera

Requires numpy, opencv-python (Debian package python3-opencv), onnxruntime;
Picamera2 only for --camera (install Debian package python3-picamera2).
"""
from __future__ import annotations

import argparse
import ast
import time
from pathlib import Path

import cv2
import numpy as np

CLASSES = ("metal", "organic", "plastic")
COLORS = ((255, 145, 35), (40, 185, 60), (30, 190, 240))  # OpenCV BGR
EXPECTED_INPUT = [1, 3, 640, 640]
EXPECTED_OUTPUT = [1, 7, 8400]


def open_model(path: Path):
    import onnxruntime as ort

    options = ort.SessionOptions()
    options.intra_op_num_threads = 4
    options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    sess = ort.InferenceSession(str(path), sess_options=options,
                                providers=["CPUExecutionProvider"])
    inputs, outputs = sess.get_inputs(), sess.get_outputs()
    if (len(inputs) != 1 or inputs[0].shape != EXPECTED_INPUT
            or inputs[0].type != "tensor(float)"):
        raise ValueError(f"Input does not match uploaded model: {[ (i.name,i.shape,i.type) for i in inputs ]}")
    if len(outputs) != 1 or outputs[0].shape != EXPECTED_OUTPUT:
        raise ValueError(f"Output does not match uploaded model: {[ (o.name,o.shape,o.type) for o in outputs ]}")
    meta = sess.get_modelmeta().custom_metadata_map
    names = ast.literal_eval(meta.get("names", "{}"))
    parsed = tuple(str(names.get(i, names.get(str(i), ""))).lower() for i in range(3))
    if parsed != CLASSES:
        raise ValueError(f"Unexpected class labels/order: {parsed}; expected {CLASSES}")
    print(f"Model loaded: {path}")
    print(f"Input: {inputs[0].name} {inputs[0].shape}")
    print(f"Output: {outputs[0].name} {outputs[0].shape}")
    print("Classes: 0=metal, 1=organic, 2=plastic")
    return sess, inputs[0].name


def prepare_image(bgr: np.ndarray):
    """Letterbox BGR frame, convert to RGB float32 NCHW 0..1."""
    original_h, original_w = bgr.shape[:2]
    size = 640
    scale = min(size / original_w, size / original_h)
    resized_w = round(original_w * scale)
    resized_h = round(original_h * scale)
    pad_w = (size - resized_w) / 2
    pad_h = (size - resized_h) / 2
    left, right = round(pad_w - 0.1), round(pad_w + 0.1)
    top, bottom = round(pad_h - 0.1), round(pad_h + 0.1)
    resized = cv2.resize(bgr, (resized_w, resized_h), interpolation=cv2.INTER_LINEAR)
    padded = cv2.copyMakeBorder(resized, top, bottom, left, right,
                                cv2.BORDER_CONSTANT, value=(114, 114, 114))
    rgb = cv2.cvtColor(padded, cv2.COLOR_BGR2RGB)
    tensor = np.ascontiguousarray(rgb.transpose(2, 0, 1)[None], dtype=np.float32)
    tensor *= 1.0 / 255.0
    return tensor, scale, left, top


def decode(raw: np.ndarray, scale: float, pad_x: int, pad_y: int,
           frame_shape: tuple, conf: float, iou: float):
    """Convert [1,7,8400] raw xywh + 3 class-score predictions to image boxes."""
    if raw.shape != (1, 7, 8400):
        raise ValueError(f"Unexpected inference output: {raw.shape}")
    rows = raw[0].T
    scores_by_class = rows[:, 4:7]
    classes = scores_by_class.argmax(axis=1)
    scores = scores_by_class[np.arange(len(rows)), classes]
    selected = np.flatnonzero(scores >= conf)
    if len(selected) == 0:
        return []
    rows = rows[selected]
    classes = classes[selected]
    scores = scores[selected]
    x_center, y_center, bw, bh = (rows[:, j] for j in range(4))
    height, width = frame_shape[:2]
    x1 = np.clip((x_center - bw / 2 - pad_x) / scale, 0, width - 1)
    y1 = np.clip((y_center - bh / 2 - pad_y) / scale, 0, height - 1)
    x2 = np.clip((x_center + bw / 2 - pad_x) / scale, 0, width - 1)
    y2 = np.clip((y_center + bh / 2 - pad_y) / scale, 0, height - 1)
    boxes = np.stack((x1, y1, x2 - x1, y2 - y1), axis=1).round().astype(int)
    results = []
    for cls in range(3):
        group = np.flatnonzero(classes == cls)
        group = [int(idx) for idx in group if boxes[idx, 2] > 0 and boxes[idx, 3] > 0]
        if not group:
            continue
        keep = cv2.dnn.NMSBoxes(
            [boxes[i].tolist() for i in group],
            [float(scores[i]) for i in group],
            score_threshold=conf,
            nms_threshold=iou,
        )
        if len(keep) == 0:
            continue
        for k in np.asarray(keep).reshape(-1):
            i = group[int(k)]
            x, y, w, h = map(int, boxes[i])
            results.append((cls, float(scores[i]), (x, y, x + w, y + h)))
    return sorted(results, key=lambda d: d[1], reverse=True)


def process(frame_bgr, session, input_name, conf, iou):
    tensor, gain, pad_x, pad_y = prepare_image(frame_bgr)
    start = time.perf_counter()
    raw = session.run(["output0"], {input_name: tensor})[0]
    infer_ms = (time.perf_counter() - start) * 1000
    detections = decode(raw, gain, pad_x, pad_y, frame_bgr.shape, conf, iou)
    annotated = frame_bgr.copy()
    for cls, score, (x1, y1, x2, y2) in detections:
        color = COLORS[cls]
        cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
        label = f"{CLASSES[cls]} {score:.2f}"
        text_y = y1 - 8 if y1 >= 25 else y1 + 22
        cv2.putText(annotated, label, (x1, text_y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.65, color, 2, cv2.LINE_AA)
    return annotated, detections, infer_ms


def main():
    p = argparse.ArgumentParser(description="AGV metal/organic/plastic detection on Raspberry Pi 5")
    p.add_argument("--model", type=Path, default=Path.home() / "agv_project/models/waste.onnx")
    mode = p.add_mutually_exclusive_group(required=True)
    mode.add_argument("--image", type=Path, help="Saved camera JPG/PNG to test")
    mode.add_argument("--camera", action="store_true", help="Use connected Pi Camera for live detections")
    p.add_argument("--conf", type=float, default=0.35, help="Confidence threshold (0..1)")
    p.add_argument("--iou", type=float, default=0.45, help="NMS IoU threshold (0..1)")
    args = p.parse_args()
    if not (0 < args.conf <= 1 and 0 < args.iou <= 1):
        p.error("--conf and --iou must be in (0,1]")
    if not args.model.is_file():
        p.error(f"Model not found: {args.model}")
    session, input_name = open_model(args.model)
    if args.image is not None:
        frame = cv2.imread(str(args.image))
        if frame is None:
            p.error(f"Cannot read image: {args.image}")
        annotated, detections, ms = process(frame, session, input_name, args.conf, args.iou)
        result = Path.home() / "agv_project/annotated_waste.jpg"
        result.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(result), annotated):
            raise RuntimeError(f"Failed to write {result}")
        print(f"Inference: {ms:.1f} ms")
        print("Detections:", [(CLASSES[c], round(s, 3)) for c, s, _ in detections])
        print("Saved annotated image:", result)
        return

    from picamera2 import Picamera2
    camera = Picamera2()
    # RGB888 camera format yields BGR-compatible arrays with Picamera2/OpenCV.
    config = camera.create_video_configuration(main={"size": (640, 480), "format": "RGB888"})
    camera.configure(config)
    camera.start()
    time.sleep(1)
    print("Camera is live. Focus the preview window and press Q to quit.")
    try:
        while True:
            frame = camera.capture_array()
            annotated, detections, ms = process(frame, session, input_name, args.conf, args.iou)
            cv2.putText(annotated, f"ONNX: {ms:.0f} ms", (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.imshow("AGV Waste Detection - YOLO26n (press Q to quit)", annotated)
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break
    except KeyboardInterrupt:
        pass
    finally:
        camera.stop()
        camera.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
