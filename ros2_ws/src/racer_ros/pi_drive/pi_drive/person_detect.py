"""
person_detect.py: YOLOv8 person detection with OpenCV only (no PyTorch / ultralytics).

Why: ultralytics pulls in PyTorch (~400-500 MB of RAM, seconds per frame on a Pi 3).
Export your .pt to ONNX once, on a PC:
    yolo export model=yolov8n.pt format=onnx imgsz=320 opset=12
and point `model_path` at the .onnx. `input_size` must equal the imgsz used for the export.

YOLOv8 ONNX output is (1, 84, N): 4 box numbers (cx, cy, w, h in model-input pixels) then
80 class scores, no objectness. Class 0 = person.
"""
from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np


@dataclass
class Detection:
    x1: float
    y1: float
    x2: float
    y2: float
    conf: float

    @property
    def area(self):
        return max(self.x2 - self.x1, 0.0) * max(self.y2 - self.y1, 0.0)


def letterbox(img, size):
    """Resize keeping aspect ratio onto a size x size gray canvas. Returns (canvas, scale, pad_x, pad_y)."""
    h, w = img.shape[:2]
    r = min(size / w, size / h)
    nw, nh = int(round(w * r)), int(round(h * r))
    resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((size, size, 3), 114, np.uint8)
    px, py = (size - nw) // 2, (size - nh) // 2
    canvas[py:py + nh, px:px + nw] = resized
    return canvas, r, px, py


def decode(output, conf_thres, nms_thres, scale, pad_x, pad_y, width, height, cls=0) -> List[Detection]:
    """Raw YOLOv8 output -> person boxes in ORIGINAL image pixels, after NMS."""
    out = np.squeeze(np.asarray(output))
    if out.ndim != 2:
        return []
    if out.shape[0] < out.shape[1]:          # (84, N) -> (N, 84)
        out = out.T
    scores = out[:, 4 + cls]
    keep = scores >= conf_thres
    if not keep.any():
        return []
    out, scores = out[keep], scores[keep]

    cx, cy, bw, bh = out[:, 0], out[:, 1], out[:, 2], out[:, 3]
    x1 = (cx - bw / 2 - pad_x) / scale
    y1 = (cy - bh / 2 - pad_y) / scale
    w, h = bw / scale, bh / scale
    boxes = np.stack([x1, y1, w, h], axis=1)
    idx = cv2.dnn.NMSBoxes(boxes.tolist(), scores.tolist(), float(conf_thres), float(nms_thres))
    dets = []
    for i in np.array(idx).reshape(-1):
        bx, by, bw_, bh_ = boxes[i]
        dets.append(Detection(float(np.clip(bx, 0, width)), float(np.clip(by, 0, height)),
                              float(np.clip(bx + bw_, 0, width)), float(np.clip(by + bh_, 0, height)),
                              float(scores[i])))
    return dets


def pick_one(dets: List[Detection], mode='largest') -> Optional[Detection]:
    """Only ONE person is reported: the largest box (closest person) or the most confident one."""
    if not dets:
        return None
    if mode == 'confident':
        return max(dets, key=lambda d: d.conf)
    return max(dets, key=lambda d: d.area)


def center_norm(det: Detection, width, height) -> Tuple[float, float]:
    """Box center as 0..1 fractions of the image (origin top-left)."""
    return (det.x1 + det.x2) / 2.0 / width, (det.y1 + det.y2) / 2.0 / height


class PersonDetector:
    def __init__(self, model_path, input_size=320, conf=0.5, nms=0.45, threads=2):
        if str(model_path).lower().endswith('.pt'):
            raise ValueError(
                f'{model_path} is a PyTorch model. Export it once on a PC: '
                '`yolo export model=yolov8n.pt format=onnx imgsz=320 opset=12` and use the .onnx')
        cv2.setNumThreads(int(threads))
        self.net = cv2.dnn.readNetFromONNX(str(model_path))
        self.size, self.conf, self.nms = int(input_size), float(conf), float(nms)

    def detect(self, frame_bgr) -> List[Detection]:
        h, w = frame_bgr.shape[:2]
        canvas, scale, px, py = letterbox(frame_bgr, self.size)
        blob = cv2.dnn.blobFromImage(canvas, 1 / 255.0, (self.size, self.size), swapRB=True, crop=False)
        self.net.setInput(blob)
        out = self.net.forward()
        return decode(out, self.conf, self.nms, scale, px, py, w, h)
