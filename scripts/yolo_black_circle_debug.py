#!/usr/bin/env python3

from __future__ import annotations

import math
import json
import mimetypes
import statistics
import time
from collections import deque
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import threading
from urllib.parse import urlparse

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from ultralytics import YOLO


INDEX_HTML = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>YOLO Black Circle Debug</title>
  <style>
    :root {
      color-scheme: dark;
      font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      background: #101215;
      color: #f3f4f6;
    }

    * {
      box-sizing: border-box;
    }

    body {
      margin: 0;
      min-height: 100vh;
      background: #101215;
    }

    main {
      min-height: 100vh;
      display: grid;
      grid-template-rows: auto minmax(0, 1fr);
    }

    header {
      display: grid;
      grid-template-columns: minmax(160px, 1fr) auto;
      gap: 12px;
      align-items: center;
      padding: 12px 16px;
      border-bottom: 1px solid #30343b;
      background: #181b20;
    }

    h1 {
      margin: 0;
      font-size: 18px;
      line-height: 1.2;
      font-weight: 700;
      letter-spacing: 0;
    }

    .stats {
      display: flex;
      flex-wrap: wrap;
      justify-content: flex-end;
      gap: 8px;
    }

    .stat {
      min-width: 82px;
      padding: 5px 8px;
      border: 1px solid #343a42;
      border-radius: 8px;
      background: #20242a;
    }

    .stat span {
      display: block;
      color: #aab2bd;
      font-size: 11px;
      line-height: 1.2;
    }

    .stat strong {
      display: block;
      color: #f7fafc;
      font-size: 16px;
      line-height: 1.2;
      font-weight: 750;
    }

    .viewer {
      min-height: 0;
      display: grid;
      place-items: center;
      padding: 12px;
      background: #0f1114;
    }

    img {
      display: block;
      max-width: 100%;
      max-height: calc(100vh - 94px);
      object-fit: contain;
      border: 1px solid #2f343b;
      background: #050607;
    }

    @media (max-width: 760px) {
      header {
        grid-template-columns: 1fr;
      }

      .stats {
        justify-content: stretch;
      }

      .stat {
        flex: 1 1 88px;
      }
    }
  </style>
</head>
<body>
  <main>
    <header>
      <h1>YOLO Black Circle Debug</h1>
      <div class="stats">
        <div class="stat"><span>Detections</span><strong id="detections">0</strong></div>
        <div class="stat"><span>Max Conf</span><strong id="confidence">0.00</strong></div>
        <div class="stat"><span>RX FPS</span><strong id="rx">0.0</strong></div>
        <div class="stat"><span>Proc FPS</span><strong id="proc">0.0</strong></div>
        <div class="stat"><span>Infer ms</span><strong id="infer">0.0</strong></div>
        <div class="stat"><span>P95 ms</span><strong id="p95">0.0</strong></div>
        <div class="stat"><span>Skipped</span><strong id="skipped">0</strong></div>
      </div>
    </header>
    <section class="viewer">
      <img id="stream" src="/stream.mjpg" alt="Live YOLO black circle detections">
    </section>
  </main>
  <script>
    const setText = (id, value) => { document.getElementById(id).textContent = value; };

    async function refreshStatus() {
      try {
        const response = await fetch("/status", {cache: "no-store"});
        if (!response.ok) return;
        const data = await response.json();
        const stats = data.stats || {};
        setText("detections", String(stats.detection_count ?? 0));
        setText("confidence", Number(stats.max_confidence ?? 0).toFixed(2));
        setText("rx", Number(stats.rx_fps ?? 0).toFixed(1));
        setText("proc", Number(stats.process_fps ?? 0).toFixed(1));
        setText("infer", Number(stats.infer_ms ?? 0).toFixed(1));
        setText("p95", Number(stats.p95_infer_ms ?? 0).toFixed(1));
        setText("skipped", String(stats.skipped_frames ?? 0));
      } catch (_) {
      }
    }

    refreshStatus();
    setInterval(refreshStatus, 500);
  </script>
</body>
</html>
"""


def resolve_model_path(model_path: str) -> Path:
    path = Path(model_path).expanduser()
    if path.is_dir():
        path = path / "weights" / "best.pt"
    return path


def fps_from_times(times: deque[float]) -> float:
    if len(times) < 2:
        return 0.0
    duration = times[-1] - times[0]
    if duration <= 0.0:
        return 0.0
    return (len(times) - 1) / duration


def percentile(values: deque[float], pct: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, math.ceil((pct / 100.0) * len(ordered)) - 1))
    return ordered[index]


def draw_text_panel(frame, lines):
    x = 10
    y = 24
    line_height = 22
    width = 0
    for line in lines:
        size, _ = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
        width = max(width, size[0])
    panel_h = line_height * len(lines) + 12
    panel_w = width + 20
    overlay = frame.copy()
    cv2.rectangle(overlay, (6, 6), (6 + panel_w, 6 + panel_h), (20, 20, 20), -1)
    cv2.addWeighted(overlay, 0.72, frame, 0.28, 0.0, frame)
    for line in lines:
        cv2.putText(frame, line, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (245, 245, 245), 1, cv2.LINE_AA)
        y += line_height


class WebDisplayState:
    def __init__(self, jpeg_quality: int):
        self.jpeg_quality = jpeg_quality
        self.lock = threading.Lock()
        self.condition = threading.Condition(self.lock)
        self.latest_frame = None
        self.latest_stats = {}
        self.latest_sequence = 0
        self.running = True

    def update(self, frame, stats):
        with self.condition:
            self.latest_frame = frame.copy()
            self.latest_stats = dict(stats)
            self.latest_sequence += 1
            self.condition.notify_all()

    def status(self):
        with self.lock:
            return {
                "has_frame": self.latest_frame is not None,
                "sequence": self.latest_sequence,
                "stats": dict(self.latest_stats),
            }

    def snapshot(self):
        with self.lock:
            if self.latest_frame is None:
                return None, self.latest_sequence, dict(self.latest_stats)
            return self.latest_frame.copy(), self.latest_sequence, dict(self.latest_stats)

    def wait_for_frame_after(self, sequence: int, timeout: float = 2.0):
        deadline = time.monotonic() + timeout
        with self.condition:
            while self.running and self.latest_sequence <= sequence:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                self.condition.wait(timeout=remaining)
            if self.latest_frame is None:
                return None, self.latest_sequence
            return self.latest_frame.copy(), self.latest_sequence

    def stop(self):
        with self.condition:
            self.running = False
            self.condition.notify_all()


def make_handler(state: WebDisplayState):
    class YoloDebugHandler(BaseHTTPRequestHandler):
        server_version = "YoloBlackCircleDebug/1.0"

        def log_message(self, fmt, *args):
            return

        def send_bytes(self, payload, content_type, status=HTTPStatus.OK):
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(payload)

        def send_json(self, payload, status=HTTPStatus.OK):
            self.send_bytes(
                json.dumps(payload).encode("utf-8"),
                "application/json; charset=utf-8",
                status,
            )

        def do_GET(self):
            parsed = urlparse(self.path)
            if parsed.path in ("/", "/index.html"):
                self.send_bytes(INDEX_HTML.encode("utf-8"), "text/html; charset=utf-8")
                return
            if parsed.path == "/status":
                self.send_json(state.status())
                return
            if parsed.path == "/snapshot.jpg":
                self.handle_snapshot()
                return
            if parsed.path == "/stream.mjpg":
                self.handle_stream()
                return
            self.send_error(HTTPStatus.NOT_FOUND)

        def handle_snapshot(self):
            frame, _, _ = state.snapshot()
            if frame is None:
                self.send_error(HTTPStatus.CONFLICT, "No frame available")
                return
            ok, encoded = cv2.imencode(
                ".jpg",
                frame,
                [cv2.IMWRITE_JPEG_QUALITY, state.jpeg_quality],
            )
            if not ok:
                self.send_error(HTTPStatus.INTERNAL_SERVER_ERROR, "JPEG encode failed")
                return
            self.send_bytes(encoded.tobytes(), mimetypes.types_map.get(".jpg", "image/jpeg"))

        def handle_stream(self):
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()

            sequence = -1
            while state.running:
                frame, sequence = state.wait_for_frame_after(sequence)
                if frame is None:
                    continue
                ok, encoded = cv2.imencode(
                    ".jpg",
                    frame,
                    [cv2.IMWRITE_JPEG_QUALITY, state.jpeg_quality],
                )
                if not ok:
                    continue
                payload = encoded.tobytes()
                try:
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(payload)}\r\n\r\n".encode("ascii"))
                    self.wfile.write(payload)
                    self.wfile.write(b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    break

    return YoloDebugHandler


class YoloBlackCircleDebugNode(Node):
    def __init__(self):
        super().__init__("yolo_black_circle_debug")

        self.declare_parameter("input_topic", "/black_feature_input_remap_node/image_remapped")
        self.declare_parameter(
            "model_path",
            str(Path.home() / "Downloads" / "train-2" / "weights" / "best.pt"),
        )
        self.declare_parameter("confidence", 0.25)
        self.declare_parameter("iou", 0.45)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("device", "cpu")
        self.declare_parameter("max_det", 20)
        self.declare_parameter("max_processing_hz", 30.0)
        self.declare_parameter("web_host", "0.0.0.0")
        self.declare_parameter("web_port", 8081)
        self.declare_parameter("jpeg_quality", 85)
        self.declare_parameter("publish_debug_image", False)
        self.declare_parameter("debug_image_topic", "~/debug_image")
        self.declare_parameter("log_interval", 30)

        self.input_topic = self.get_parameter("input_topic").value
        self.confidence = float(self.get_parameter("confidence").value)
        self.iou = float(self.get_parameter("iou").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.device = str(self.get_parameter("device").value)
        self.max_det = int(self.get_parameter("max_det").value)
        self.web_host = str(self.get_parameter("web_host").value)
        self.web_port = int(self.get_parameter("web_port").value)
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self.publish_debug_image = bool(self.get_parameter("publish_debug_image").value)
        self.log_interval = max(1, int(self.get_parameter("log_interval").value))
        max_processing_hz = float(self.get_parameter("max_processing_hz").value)
        self.jpeg_quality = min(100, max(1, self.jpeg_quality))

        self.model_path = resolve_model_path(str(self.get_parameter("model_path").value))
        if not self.model_path.exists():
            raise FileNotFoundError(
                f"YOLO model not found: {self.model_path}. "
                "Pass model_path:=/path/to/train-2 or /path/to/best.pt."
            )

        self.get_logger().info(f"Loading YOLO model: {self.model_path}")
        load_start = time.perf_counter()
        self.model = YOLO(str(self.model_path))
        self.get_logger().info(f"Loaded model in {(time.perf_counter() - load_start) * 1000.0:.1f} ms")
        self.get_logger().info(
            "Input topic=%s confidence=%.2f iou=%.2f imgsz=%d device=%s max_det=%d"
            % (self.input_topic, self.confidence, self.iou, self.imgsz, self.device, self.max_det)
        )

        self.bridge = CvBridge()
        self.latest_frame = None
        self.latest_stamp = None
        self.latest_recv_time = 0.0
        self.latest_sequence = 0
        self.processed_sequence = 0
        self.skipped_frames = 0
        self.frame_shape = None

        self.rx_times: deque[float] = deque(maxlen=120)
        self.process_times: deque[float] = deque(maxlen=120)
        self.infer_ms: deque[float] = deque(maxlen=120)
        self.total_ms: deque[float] = deque(maxlen=120)
        self.web_state = WebDisplayState(self.jpeg_quality)
        self.web_server = ThreadingHTTPServer(
            (self.web_host, self.web_port),
            make_handler(self.web_state),
        )
        self.web_thread = threading.Thread(
            target=self.web_server.serve_forever,
            name="yolo_black_circle_debug_http",
            daemon=True,
        )
        self.web_thread.start()
        if self.web_host in ("", "0.0.0.0", "::"):
            self.get_logger().info(
                f"Web viewer listening on http://0.0.0.0:{self.web_port}/ "
                f"(open http://<robot-ip>:{self.web_port}/ from your browser)"
            )
        else:
            self.get_logger().info(f"Web viewer listening on http://{self.web_host}:{self.web_port}/")

        self.subscription = self.create_subscription(
            Image,
            self.input_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.debug_publisher = None
        if self.publish_debug_image:
            self.debug_publisher = self.create_publisher(
                Image,
                str(self.get_parameter("debug_image_topic").value),
                10,
            )

        timer_period = 0.001
        if max_processing_hz > 0.0:
            timer_period = 1.0 / max_processing_hz
        self.timer = self.create_timer(timer_period, self.process_latest_frame)

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"Failed to convert image: {exc}")
            return

        now = time.perf_counter()
        self.latest_frame = frame
        self.latest_stamp = msg.header.stamp
        self.latest_recv_time = now
        self.latest_sequence += 1
        self.rx_times.append(now)

        if self.frame_shape != frame.shape:
            self.frame_shape = frame.shape
            self.get_logger().info(f"Receiving frames from {self.input_topic}: {frame.shape[1]}x{frame.shape[0]}")

    def process_latest_frame(self):
        if self.latest_frame is None:
            return
        if self.latest_sequence == self.processed_sequence:
            return

        sequence = self.latest_sequence
        skipped_now = max(0, sequence - self.processed_sequence - 1)
        self.skipped_frames += skipped_now
        frame = self.latest_frame.copy()
        recv_time = self.latest_recv_time
        self.processed_sequence = sequence

        total_start = time.perf_counter()
        results = self.model.predict(
            source=frame,
            conf=self.confidence,
            iou=self.iou,
            imgsz=self.imgsz,
            device=self.device,
            max_det=self.max_det,
            verbose=False,
        )
        result = results[0]
        yolo_speed = getattr(result, "speed", {}) or {}
        infer_elapsed_ms = float(yolo_speed.get("inference", 0.0))
        detections = self.draw_detections(frame, result)
        total_elapsed_ms = (time.perf_counter() - total_start) * 1000.0

        now = time.perf_counter()
        self.process_times.append(now)
        self.infer_ms.append(infer_elapsed_ms)
        self.total_ms.append(total_elapsed_ms)

        max_conf = max((det["confidence"] for det in detections), default=0.0)
        stats = self.make_stats(len(detections), max_conf, now - recv_time)
        self.draw_stats(frame, stats)
        self.web_state.update(frame, stats)

        if self.debug_publisher is not None:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            if self.latest_stamp is not None:
                msg.header.stamp = self.latest_stamp
            msg.header.frame_id = "yolo_black_circle_debug"
            self.debug_publisher.publish(msg)

        if len(self.process_times) % self.log_interval == 0:
            self.log_stats(sequence, len(detections), max_conf, skipped_now)

    def draw_detections(self, frame, result):
        detections = []
        names = getattr(result, "names", {}) or getattr(self.model, "names", {})
        boxes = getattr(result, "boxes", None)
        if boxes is None or len(boxes) == 0:
            return detections

        xyxy = boxes.xyxy.cpu().numpy()
        confs = boxes.conf.cpu().numpy()
        classes = boxes.cls.cpu().numpy().astype(int)
        for bounds, confidence, class_id in zip(xyxy, confs, classes):
            x1, y1, x2, y2 = [int(round(v)) for v in bounds]
            label = names.get(class_id, str(class_id)) if isinstance(names, dict) else str(class_id)
            detections.append(
                {
                    "bounds": (x1, y1, x2, y2),
                    "confidence": float(confidence),
                    "class_id": int(class_id),
                    "label": label,
                }
            )
            color = (0, 255, 120)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            center = ((x1 + x2) // 2, (y1 + y2) // 2)
            radius = max(2, min(x2 - x1, y2 - y1) // 2)
            cv2.circle(frame, center, radius, (0, 180, 255), 1)
            text = f"{label} {confidence:.2f}"
            text_size, baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
            text_y = max(18, y1 - 7)
            cv2.rectangle(
                frame,
                (x1, text_y - text_size[1] - baseline - 4),
                (x1 + text_size[0] + 6, text_y + baseline),
                (0, 90, 55),
                -1,
            )
            cv2.putText(frame, text, (x1 + 3, text_y - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
        return detections

    def make_stats(self, detection_count: int, max_conf: float, frame_age_s: float):
        avg_infer = statistics.fmean(self.infer_ms) if self.infer_ms else 0.0
        avg_total = statistics.fmean(self.total_ms) if self.total_ms else 0.0
        return {
            "detection_count": detection_count,
            "max_confidence": max_conf,
            "rx_fps": fps_from_times(self.rx_times),
            "process_fps": fps_from_times(self.process_times),
            "infer_ms": self.infer_ms[-1] if self.infer_ms else 0.0,
            "avg_infer_ms": avg_infer,
            "p95_infer_ms": percentile(self.infer_ms, 95),
            "total_ms": self.total_ms[-1] if self.total_ms else 0.0,
            "avg_total_ms": avg_total,
            "frame_age_ms": frame_age_s * 1000.0,
            "skipped_frames": self.skipped_frames,
            "confidence_threshold": self.confidence,
            "imgsz": self.imgsz,
            "device": self.device,
        }

    def draw_stats(self, frame, stats):
        lines = [
            f"YOLO black circles: {stats['detection_count']}  max_conf: {stats['max_confidence']:.2f}",
            f"rx_fps: {stats['rx_fps']:.1f}  proc_fps: {stats['process_fps']:.1f}",
            f"infer_ms: {stats['infer_ms']:.1f} avg {stats['avg_infer_ms']:.1f} p95 {stats['p95_infer_ms']:.1f}",
            f"total_ms: {stats['total_ms']:.1f} avg {stats['avg_total_ms']:.1f}",
            f"frame_age_ms: {stats['frame_age_ms']:.1f}  skipped: {stats['skipped_frames']}",
            f"conf: {stats['confidence_threshold']:.2f} imgsz: {stats['imgsz']} device: {stats['device']}",
        ]
        draw_text_panel(frame, lines)

    def log_stats(self, sequence: int, detection_count: int, max_conf: float, skipped_now: int):
        avg_infer = statistics.fmean(self.infer_ms) if self.infer_ms else 0.0
        avg_total = statistics.fmean(self.total_ms) if self.total_ms else 0.0
        self.get_logger().info(
            "seq=%d detections=%d max_conf=%.2f infer_ms=%.1f avg_infer_ms=%.1f "
            "p95_infer_ms=%.1f total_ms=%.1f avg_total_ms=%.1f rx_fps=%.1f "
            "proc_fps=%.1f skipped_now=%d skipped_total=%d"
            % (
                sequence,
                detection_count,
                max_conf,
                self.infer_ms[-1] if self.infer_ms else 0.0,
                avg_infer,
                percentile(self.infer_ms, 95),
                self.total_ms[-1] if self.total_ms else 0.0,
                avg_total,
                fps_from_times(self.rx_times),
                fps_from_times(self.process_times),
                skipped_now,
                self.skipped_frames,
            )
        )

    def destroy_node(self):
        self.web_state.stop()
        self.web_server.shutdown()
        self.web_server.server_close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloBlackCircleDebugNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
