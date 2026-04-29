#!/usr/bin/env python3

import argparse
import json
import mimetypes
import re
import signal
import sys
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Image


INDEX_HTML = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Remapped Capture</title>
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
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 16px;
      padding: 12px 16px;
      border-bottom: 1px solid #2f343b;
      background: #181b20;
    }

    h1 {
      margin: 0;
      font-size: 18px;
      font-weight: 650;
      letter-spacing: 0;
      white-space: nowrap;
    }

    .toolbar {
      display: flex;
      align-items: center;
      justify-content: flex-end;
      gap: 10px;
      min-width: 0;
    }

    .stat {
      display: grid;
      gap: 1px;
      min-width: 72px;
      padding: 4px 8px;
      border: 1px solid #343a42;
      border-radius: 8px;
      background: #20242a;
    }

    .stat span {
      color: #a9b1bd;
      font-size: 11px;
      line-height: 1.2;
    }

    .stat strong {
      font-size: 16px;
      line-height: 1.1;
    }

    button {
      appearance: none;
      border: 1px solid #538f7e;
      border-radius: 8px;
      background: #d7fff2;
      color: #0e2b22;
      min-height: 40px;
      padding: 0 14px;
      font: inherit;
      font-weight: 750;
      cursor: pointer;
    }

    button:active {
      transform: translateY(1px);
    }

    button[disabled] {
      opacity: 0.55;
      cursor: wait;
    }

    .viewer {
      min-height: 0;
      display: grid;
      place-items: center;
      padding: 12px;
      background:
        linear-gradient(45deg, #15181d 25%, transparent 25%),
        linear-gradient(-45deg, #15181d 25%, transparent 25%),
        linear-gradient(45deg, transparent 75%, #15181d 75%),
        linear-gradient(-45deg, transparent 75%, #15181d 75%);
      background-color: #0f1114;
      background-position: 0 0, 0 8px, 8px -8px, -8px 0;
      background-size: 16px 16px;
    }

    .stage {
      width: 100%;
      height: 100%;
      display: grid;
      place-items: center;
      overflow: hidden;
    }

    img {
      display: block;
      max-width: 100%;
      max-height: calc(100vh - 88px);
      object-fit: contain;
      image-rendering: auto;
      border: 1px solid #2f343b;
      background: #050607;
    }

    .toast {
      position: fixed;
      left: 50%;
      bottom: 18px;
      transform: translateX(-50%);
      max-width: min(560px, calc(100vw - 24px));
      padding: 10px 14px;
      border-radius: 8px;
      border: 1px solid #3d4650;
      background: #1f242b;
      color: #f3f4f6;
      box-shadow: 0 10px 30px rgba(0, 0, 0, 0.28);
      opacity: 0;
      pointer-events: none;
      transition: opacity 140ms ease;
      overflow-wrap: anywhere;
    }

    .toast.show {
      opacity: 1;
    }

    @media (max-width: 720px) {
      header {
        align-items: stretch;
        flex-direction: column;
      }

      h1 {
        white-space: normal;
      }

      .toolbar {
        justify-content: stretch;
      }

      .stat {
        flex: 1 1 0;
      }

      button {
        flex: 1 0 auto;
      }
    }
  </style>
</head>
<body>
  <main>
    <header>
      <h1>Remapped Capture</h1>
      <div class="toolbar">
        <div class="stat"><span>Saved</span><strong id="saved">0</strong></div>
        <div class="stat"><span>Target</span><strong id="target">300</strong></div>
        <div class="stat"><span>FPS</span><strong id="fps">0.0</strong></div>
        <button id="save" type="button" autofocus>Save</button>
      </div>
    </header>
    <section class="viewer">
      <div class="stage">
        <img id="stream" src="/stream.mjpg" alt="Live remapped camera">
      </div>
    </section>
  </main>
  <div class="toast" id="toast"></div>
  <script>
    const savedEl = document.getElementById("saved");
    const targetEl = document.getElementById("target");
    const fpsEl = document.getElementById("fps");
    const saveButton = document.getElementById("save");
    const toast = document.getElementById("toast");
    let saving = false;
    let toastTimer = 0;

    function showToast(message) {
      toast.textContent = message;
      toast.classList.add("show");
      clearTimeout(toastTimer);
      toastTimer = setTimeout(() => toast.classList.remove("show"), 1400);
    }

    function updateStatus(data) {
      savedEl.textContent = data.saved_count ?? 0;
      targetEl.textContent = data.target_count ?? 300;
      fpsEl.textContent = Number(data.fps ?? 0).toFixed(1);
    }

    async function refreshStatus() {
      try {
        const response = await fetch("/status", {cache: "no-store"});
        if (response.ok) updateStatus(await response.json());
      } catch (_) {
      }
    }

    async function saveFrame() {
      if (saving) return;
      saving = true;
      saveButton.disabled = true;
      try {
        const response = await fetch("/save", {method: "POST"});
        const data = await response.json();
        if (!response.ok || !data.saved) {
          throw new Error(data.error || "No frame available");
        }
        updateStatus(data);
        showToast(data.basename);
      } catch (error) {
        showToast(error.message);
      } finally {
        saving = false;
        saveButton.disabled = false;
        saveButton.focus();
      }
    }

    document.addEventListener("keydown", (event) => {
      if (event.key && event.key.toLowerCase() === "s" && !event.repeat) {
        event.preventDefault();
        saveFrame();
      }
    });

    saveButton.addEventListener("click", saveFrame);
    refreshStatus();
    setInterval(refreshStatus, 1000);
  </script>
</body>
</html>
"""


class CaptureState:
    def __init__(self, output_dir, prefix, image_format, target_count, jpeg_quality):
        self.output_dir = output_dir
        self.prefix = prefix
        self.image_format = image_format.lower()
        self.target_count = target_count
        self.jpeg_quality = jpeg_quality
        self.lock = threading.Lock()
        self.condition = threading.Condition(self.lock)
        self.save_lock = threading.Lock()
        self.latest_frame = None
        self.latest_stamp = None
        self.frame_sequence = 0
        self.saved_count = self._find_existing_count()
        self.last_save_path = None
        self.frame_times = []
        self.running = True

    def _find_existing_count(self):
        pattern = re.compile(rf"^{re.escape(self.prefix)}_(\d+)\.{re.escape(self.image_format)}$")
        max_index = 0
        if not self.output_dir.exists():
            return max_index
        for path in self.output_dir.iterdir():
            match = pattern.match(path.name)
            if match:
                max_index = max(max_index, int(match.group(1)))
        return max_index

    def update_frame(self, frame, stamp):
        now = time.monotonic()
        with self.condition:
            self.latest_frame = frame.copy()
            self.latest_stamp = stamp
            self.frame_sequence += 1
            self.frame_times.append(now)
            cutoff = now - 2.0
            while self.frame_times and self.frame_times[0] < cutoff:
                self.frame_times.pop(0)
            self.condition.notify_all()

    def fps(self):
        if len(self.frame_times) < 2:
            return 0.0
        duration = self.frame_times[-1] - self.frame_times[0]
        if duration <= 0.0:
            return 0.0
        return (len(self.frame_times) - 1) / duration

    def snapshot(self):
        with self.lock:
            if self.latest_frame is None:
                return None, None, self.frame_sequence
            return self.latest_frame.copy(), self.latest_stamp, self.frame_sequence

    def wait_for_frame_after(self, sequence, timeout=2.0):
        deadline = time.monotonic() + timeout
        with self.condition:
            while self.running and self.frame_sequence <= sequence:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                self.condition.wait(timeout=remaining)
            if self.latest_frame is None:
                return None, None, self.frame_sequence
            return self.latest_frame.copy(), self.latest_stamp, self.frame_sequence

    def save_latest(self):
        with self.save_lock:
            with self.lock:
                if self.latest_frame is None:
                    raise RuntimeError("Waiting for first frame")
                frame = self.latest_frame.copy()
                index = self.saved_count + 1

            self.output_dir.mkdir(parents=True, exist_ok=True)
            path = self.output_dir / f"{self.prefix}_{index:04d}.{self.image_format}"
            while path.exists():
                index += 1
                path = self.output_dir / f"{self.prefix}_{index:04d}.{self.image_format}"
            if self.image_format in ("jpg", "jpeg"):
                params = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
            elif self.image_format == "png":
                params = [cv2.IMWRITE_PNG_COMPRESSION, 3]
            else:
                params = []
            if not cv2.imwrite(str(path), frame, params):
                raise RuntimeError(f"Failed to write {path}")

            with self.lock:
                self.saved_count = index
                self.last_save_path = path
                return self.status(
                    extra={"saved": True, "path": str(path), "basename": path.name}
                )

    def status(self, extra=None):
        data = {
            "has_frame": self.latest_frame is not None,
            "frame_sequence": self.frame_sequence,
            "saved_count": self.saved_count,
            "target_count": self.target_count,
            "fps": self.fps(),
            "output_dir": str(self.output_dir),
            "last_save_path": str(self.last_save_path) if self.last_save_path else None,
        }
        if extra:
            data.update(extra)
        return data


class RemappedPhotoCaptureNode(Node):
    def __init__(self, state, input_topic):
        super().__init__("remapped_photo_capture_ui")
        self.state = state
        self.bridge = CvBridge()
        self.declare_parameter("input_topic", input_topic)
        self.input_topic = input_topic
        self.subscription = self.create_subscription(
            Image,
            self.input_topic,
            self.image_callback,
            10,
        )
        self.get_logger().info(f"Subscribing to remapped image topic: {self.input_topic}")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"Failed to convert image: {exc}")
            return
        self.state.update_frame(frame, msg.header.stamp)


def make_handler(state):
    class CaptureHandler(BaseHTTPRequestHandler):
        server_version = "RemappedCapture/1.0"

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
                with state.lock:
                    payload = state.status()
                self.send_json(payload)
                return
            if parsed.path == "/save":
                self.handle_save()
                return
            if parsed.path == "/snapshot.jpg":
                self.handle_snapshot()
                return
            if parsed.path == "/stream.mjpg":
                self.handle_stream()
                return
            self.send_error(HTTPStatus.NOT_FOUND)

        def do_POST(self):
            parsed = urlparse(self.path)
            if parsed.path == "/save":
                self.handle_save()
                return
            self.send_error(HTTPStatus.NOT_FOUND)

        def handle_save(self):
            try:
                self.send_json(state.save_latest())
            except RuntimeError as exc:
                with state.lock:
                    payload = state.status(extra={"saved": False, "error": str(exc)})
                self.send_json(payload, HTTPStatus.CONFLICT)

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
                frame, _, sequence = state.wait_for_frame_after(sequence)
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
                except (BrokenPipeError, ConnectionResetError, TimeoutError):
                    break

    return CaptureHandler


def parse_args(argv=None):
    if argv is None:
        argv = sys.argv
    parser = argparse.ArgumentParser(
        description="Serve a browser capture UI for a remapped ROS image topic."
    )
    parser.add_argument(
        "--input-topic",
        default="/black_feature_input_remap_node/image_remapped",
        help="Remapped sensor_msgs/Image topic to preview and save.",
    )
    parser.add_argument(
        "--output-dir",
        default="~/rcj_remapped_captures",
        help="Directory on this machine where captured images are saved.",
    )
    parser.add_argument("--host", default="0.0.0.0", help="HTTP bind address.")
    parser.add_argument("--port", type=int, default=8080, help="HTTP port.")
    parser.add_argument("--prefix", default="remapped", help="Saved image filename prefix.")
    parser.add_argument(
        "--format",
        choices=("png", "jpg", "jpeg"),
        default="png",
        help="Saved image format.",
    )
    parser.add_argument("--target-count", type=int, default=300, help="Capture target shown in the UI.")
    parser.add_argument("--jpeg-quality", type=int, default=85, help="MJPEG/snapshot JPEG quality.")
    return parser.parse_args(remove_ros_args(args=argv)[1:])


def main():
    args = parse_args()
    output_dir = Path(args.output_dir).expanduser().resolve()
    state = CaptureState(
        output_dir=output_dir,
        prefix=args.prefix,
        image_format=args.format,
        target_count=max(1, args.target_count),
        jpeg_quality=min(100, max(1, args.jpeg_quality)),
    )

    rclpy.init(args=sys.argv)
    node = RemappedPhotoCaptureNode(state, args.input_topic)

    httpd = ThreadingHTTPServer((args.host, args.port), make_handler(state))
    httpd.daemon_threads = True

    def stop_http():
        state.running = False
        with state.condition:
            state.condition.notify_all()
        httpd.shutdown()

    def handle_signal(_signum=None, _frame=None):
        stop_http()
        rclpy.try_shutdown()

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    server_thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    server_thread.start()
    node.get_logger().info(f"Capture UI: http://{args.host}:{args.port}")
    node.get_logger().info(f"Saving images under: {output_dir}")

    try:
        rclpy.spin(node)
    finally:
        stop_http()
        server_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
