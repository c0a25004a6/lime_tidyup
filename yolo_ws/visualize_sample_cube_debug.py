#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont
from ultralytics import YOLO

VERTEX_LABELS = (
    "x-_y-_z-",
    "x+_y-_z-",
    "x+_y+_z-",
    "x-_y+_z-",
    "x-_y-_z+",
    "x+_y-_z+",
    "x+_y+_z+",
    "x-_y+_z+",
)

CUBE_EDGES = (
    (0, 1), (1, 2), (2, 3), (3, 0),
    (4, 5), (5, 6), (6, 7), (7, 4),
    (0, 4), (1, 5), (2, 6), (3, 7),
)

COLORS = (
    (255, 80, 80),
    (80, 190, 255),
    (80, 235, 110),
    (255, 200, 70),
    (200, 100, 255),
    (255, 100, 200),
    (90, 255, 225),
    (245, 245, 245),
)


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def text_with_bg(draw: ImageDraw.ImageDraw, xy, text: str, *, fill=(255, 255, 255), bg=(0, 0, 0)):
    font = ImageFont.load_default()
    x, y = xy
    bbox = draw.textbbox((x, y), text, font=font, stroke_width=1)
    pad = 3
    draw.rectangle((bbox[0] - pad, bbox[1] - pad, bbox[2] + pad, bbox[3] + pad), fill=bg)
    draw.text((x, y), text, fill=fill, font=font, stroke_width=1, stroke_fill=(0, 0, 0))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", default="sample.png")
    parser.add_argument("--model", default="runs/pose/train-2/weights/best.pt")
    parser.add_argument("--out-image", default="sample_prediction_debug.jpg")
    parser.add_argument("--out-json", default="sample_prediction_debug.json")
    parser.add_argument("--conf", type=float, default=0.25)
    args = parser.parse_args()

    image_path = Path(args.image).resolve()
    model_path = Path(args.model).resolve()
    out_image = Path(args.out_image).resolve()
    out_json = Path(args.out_json).resolve()

    model = YOLO(str(model_path))
    result = model.predict(source=str(image_path), conf=args.conf, save=False, verbose=False, device="cpu")[0]

    image = Image.open(image_path).convert("RGB")
    draw = ImageDraw.Draw(image)
    width, height = image.size

    boxes = [] if result.boxes is None else result.boxes.xyxy.cpu().tolist()
    classes = [] if result.boxes is None else result.boxes.cls.cpu().tolist()
    confs = [] if result.boxes is None else result.boxes.conf.cpu().tolist()
    keypoints_xy = [] if result.keypoints is None else result.keypoints.xy.cpu().tolist()
    if result.keypoints is None or result.keypoints.conf is None:
        keypoints_conf = [[1.0] * len(points) for points in keypoints_xy]
    else:
        keypoints_conf = result.keypoints.conf.cpu().tolist()

    detections = []
    for det_idx, box in enumerate(boxes):
        x1, y1, x2, y2 = map(float, box)
        cls_id = int(classes[det_idx])
        cls_name = str(result.names.get(cls_id, cls_id))
        det_conf = float(confs[det_idx])
        draw.rectangle((x1, y1, x2, y2), outline=(255, 255, 255), width=3)
        text_with_bg(draw, (x1 + 5, max(3, y1 + 5)), f"{cls_name} {det_conf:.3f}")

        points = keypoints_xy[det_idx] if det_idx < len(keypoints_xy) else []
        scores = keypoints_conf[det_idx] if det_idx < len(keypoints_conf) else []

        if len(points) == 8:
            for a, b in CUBE_EDGES:
                ax, ay = points[a]
                bx, by = points[b]
                if (ax, ay) != (0.0, 0.0) and (bx, by) != (0.0, 0.0):
                    draw.line((ax, ay, bx, by), fill=(255, 235, 0), width=2)

        point_records = []
        for idx, point in enumerate(points):
            x, y = map(float, point)
            score = float(scores[idx]) if idx < len(scores) else 1.0
            label = VERTEX_LABELS[idx] if idx < len(VERTEX_LABELS) else f"k{idx}"
            color = COLORS[idx % len(COLORS)]
            radius = 5
            draw.ellipse((x - radius, y - radius, x + radius, y + radius), fill=color, outline=(0, 0, 0), width=2)
            text_with_bg(draw, (x + 7, y - 8), f"k{idx} {score:.2f}", fill=color)
            point_records.append({"index": idx, "vertex": label, "xy": [x, y], "confidence": score})

        detections.append({
            "index": det_idx,
            "class_id": cls_id,
            "class_name": cls_name,
            "confidence": det_conf,
            "box_xyxy": [x1, y1, x2, y2],
            "keypoints": point_records,
        })

    panel = [
        "YOLO Pose debug",
        f"image: {image_path.name} ({width}x{height})",
        f"model: {model_path.name}",
        f"det conf threshold: {args.conf:.2f}",
        f"detections: {len(detections)}",
    ]
    if detections:
        panel.append(f"top conf: {max(d['confidence'] for d in detections):.3f}")
        panel.append(f"keypoints: {len(detections[0]['keypoints'])}")
    panel_x = 8
    panel_y = 8
    for line_idx, line in enumerate(panel):
        text_with_bg(draw, (panel_x, panel_y + line_idx * 16), line, fill=(235, 255, 235), bg=(15, 15, 15))

    max_width = 960
    if image.width > max_width:
        new_height = round(image.height * max_width / image.width)
        image = image.resize((max_width, new_height), Image.Resampling.LANCZOS)

    out_image.parent.mkdir(parents=True, exist_ok=True)
    image.save(out_image, format="JPEG", quality=82, optimize=True)

    payload = {
        "schema_version": "lime-dev2-sample-yolo-pose-v1",
        "source": {
            "image": str(image_path.name),
            "image_sha256": sha256(image_path),
            "model": str(model_path.name),
            "model_sha256": sha256(model_path),
        },
        "runtime": {
            "confidence_threshold": args.conf,
            "device": "cpu",
            "camera_info_used": False,
            "pnp_pose_estimation_performed": False,
        },
        "detections": detections,
        "note": "Visualization is 2D YOLO Pose inference only. No CameraInfo was supplied, so no 3D pose axes or PnP claim is drawn.",
    }
    out_json.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
