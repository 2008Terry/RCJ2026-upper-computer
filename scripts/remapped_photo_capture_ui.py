#!/usr/bin/env python3

import argparse
import json
import mimetypes
import re
import signal
import shutil
import sys
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
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


MASK_EDITOR_HTML = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Robot Mask Editor</title>
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
      gap: 14px;
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
      gap: 8px;
      min-width: 0;
      flex-wrap: wrap;
    }

    .tabs {
      display: inline-flex;
      gap: 2px;
      padding: 3px;
      border: 1px solid #343a42;
      border-radius: 8px;
      background: #20242a;
    }

    button {
      appearance: none;
      border: 1px solid #414954;
      border-radius: 7px;
      background: #242a31;
      color: #f3f4f6;
      min-height: 38px;
      padding: 0 12px;
      font: inherit;
      font-weight: 700;
      cursor: pointer;
    }

    button.primary {
      border-color: #538f7e;
      background: #d7fff2;
      color: #0e2b22;
    }

    button.active {
      border-color: #7fb0ff;
      background: #d7e7ff;
      color: #10243f;
    }

    button:active {
      transform: translateY(1px);
    }

    button[disabled] {
      opacity: 0.55;
      cursor: wait;
    }

    .stat {
      display: grid;
      gap: 1px;
      min-width: 68px;
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
      font-size: 15px;
      line-height: 1.1;
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
      position: relative;
      display: inline-block;
      max-width: 100%;
      max-height: calc(100vh - 88px);
      line-height: 0;
      cursor: crosshair;
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

    canvas {
      position: absolute;
      inset: 0;
      width: 100%;
      height: 100%;
    }

    .toast {
      position: fixed;
      left: 50%;
      bottom: 18px;
      transform: translateX(-50%);
      max-width: min(720px, calc(100vw - 24px));
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

    @media (max-width: 760px) {
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

      .tabs,
      button,
      .stat {
        flex: 1 1 auto;
      }
    }
  </style>
</head>
<body>
  <main>
    <header>
      <h1>Robot Mask Editor</h1>
      <div class="toolbar">
        <div class="tabs">
          <button id="rawTab" type="button" class="active">Raw</button>
          <button id="remappedTab" type="button">Remapped</button>
        </div>
        <div class="stat"><span>Points</span><strong id="pointCount">0</strong></div>
        <div class="stat"><span>FPS</span><strong id="fps">0.0</strong></div>
        <button id="undo" type="button">Undo</button>
        <button id="clear" type="button">Clear</button>
        <button id="saveMask" type="button" class="primary">Save Mask</button>
      </div>
    </header>
    <section class="viewer">
      <div class="stage" id="stage">
        <img id="stream" src="/stream.mjpg?stream=raw" alt="Live camera stream">
        <canvas id="overlay"></canvas>
      </div>
    </section>
  </main>
  <div class="toast" id="toast"></div>
  <script>
    const streams = {
      raw: {label: "Raw", points: []},
      remapped: {label: "Remapped", points: []},
    };
    let activeStream = "raw";
    let saving = false;
    let toastTimer = 0;

    const streamImg = document.getElementById("stream");
    const canvas = document.getElementById("overlay");
    const ctx = canvas.getContext("2d");
    const pointCount = document.getElementById("pointCount");
    const fpsEl = document.getElementById("fps");
    const rawTab = document.getElementById("rawTab");
    const remappedTab = document.getElementById("remappedTab");
    const undoButton = document.getElementById("undo");
    const clearButton = document.getElementById("clear");
    const saveMaskButton = document.getElementById("saveMask");
    const toast = document.getElementById("toast");

    function showToast(message) {
      toast.textContent = message;
      toast.classList.add("show");
      clearTimeout(toastTimer);
      toastTimer = setTimeout(() => toast.classList.remove("show"), 1800);
    }

    function currentPoints() {
      return streams[activeStream].points;
    }

    function resizeCanvas() {
      const rect = streamImg.getBoundingClientRect();
      const width = Math.max(1, Math.round(rect.width));
      const height = Math.max(1, Math.round(rect.height));
      if (canvas.width !== width || canvas.height !== height) {
        canvas.width = width;
        canvas.height = height;
      }
      drawOverlay();
    }

    function imageToCanvas(point) {
      const naturalWidth = streamImg.naturalWidth || canvas.width;
      const naturalHeight = streamImg.naturalHeight || canvas.height;
      return {
        x: point[0] * canvas.width / naturalWidth,
        y: point[1] * canvas.height / naturalHeight,
      };
    }

    function canvasToImage(clientX, clientY) {
      const rect = canvas.getBoundingClientRect();
      const naturalWidth = streamImg.naturalWidth || canvas.width;
      const naturalHeight = streamImg.naturalHeight || canvas.height;
      const x = (clientX - rect.left) * naturalWidth / rect.width;
      const y = (clientY - rect.top) * naturalHeight / rect.height;
      return [
        Math.max(0, Math.min(naturalWidth - 1, Math.round(x))),
        Math.max(0, Math.min(naturalHeight - 1, Math.round(y))),
      ];
    }

    function drawOverlay() {
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      const points = currentPoints();
      if (points.length >= 3) {
        ctx.beginPath();
        points.map(imageToCanvas).forEach((point, index) => {
          if (index === 0) ctx.moveTo(point.x, point.y);
          else ctx.lineTo(point.x, point.y);
        });
        ctx.closePath();
        ctx.fillStyle = "rgba(255, 40, 40, 0.28)";
        ctx.fill();
      }
      if (points.length >= 2) {
        ctx.beginPath();
        points.map(imageToCanvas).forEach((point, index) => {
          if (index === 0) ctx.moveTo(point.x, point.y);
          else ctx.lineTo(point.x, point.y);
        });
        if (points.length >= 3) ctx.closePath();
        ctx.strokeStyle = "rgb(255, 230, 80)";
        ctx.lineWidth = 2;
        ctx.stroke();
      }
      points.forEach((rawPoint, index) => {
        const point = imageToCanvas(rawPoint);
        ctx.beginPath();
        ctx.arc(point.x, point.y, 5, 0, Math.PI * 2);
        ctx.fillStyle = "rgb(255, 60, 60)";
        ctx.fill();
        ctx.font = "14px sans-serif";
        ctx.lineWidth = 3;
        ctx.strokeStyle = "rgba(0, 0, 0, 0.8)";
        ctx.strokeText(String(index + 1), point.x + 8, point.y - 8);
        ctx.fillStyle = "rgb(215, 255, 242)";
        ctx.fillText(String(index + 1), point.x + 8, point.y - 8);
      });
      pointCount.textContent = points.length;
    }

    function setStream(streamName) {
      activeStream = streamName;
      rawTab.classList.toggle("active", streamName === "raw");
      remappedTab.classList.toggle("active", streamName === "remapped");
      streamImg.src = `/stream.mjpg?stream=${streamName}&t=${Date.now()}`;
      drawOverlay();
    }

    async function refreshStatus() {
      try {
        const response = await fetch("/status", {cache: "no-store"});
        if (!response.ok) return;
        const data = await response.json();
        fpsEl.textContent = Number(data.streams?.[activeStream]?.fps ?? 0).toFixed(1);
      } catch (_) {
      }
    }

    async function saveMask() {
      if (saving) return;
      const points = currentPoints();
      if (points.length < 3) {
        showToast("Need at least 3 points");
        return;
      }
      saving = true;
      saveMaskButton.disabled = true;
      try {
        const response = await fetch("/mask", {
          method: "POST",
          headers: {"Content-Type": "application/json"},
          body: JSON.stringify({stream: activeStream, points}),
        });
        const data = await response.json();
        if (!response.ok || !data.saved) {
          throw new Error(data.error || "Failed to save mask");
        }
        showToast(`${streams[activeStream].label} mask saved: ${data.mask_path}`);
      } catch (error) {
        showToast(error.message);
      } finally {
        saving = false;
        saveMaskButton.disabled = false;
        saveMaskButton.focus();
      }
    }

    canvas.addEventListener("click", (event) => {
      currentPoints().push(canvasToImage(event.clientX, event.clientY));
      drawOverlay();
    });

    canvas.addEventListener("contextmenu", (event) => {
      event.preventDefault();
      currentPoints().pop();
      drawOverlay();
    });

    rawTab.addEventListener("click", () => setStream("raw"));
    remappedTab.addEventListener("click", () => setStream("remapped"));
    undoButton.addEventListener("click", () => {
      currentPoints().pop();
      drawOverlay();
    });
    clearButton.addEventListener("click", () => {
      currentPoints().length = 0;
      drawOverlay();
    });
    saveMaskButton.addEventListener("click", saveMask);

    document.addEventListener("keydown", (event) => {
      if (event.repeat) return;
      const key = event.key.toLowerCase();
      if (key === "z") {
        currentPoints().pop();
        drawOverlay();
      } else if (key === "c") {
        currentPoints().length = 0;
        drawOverlay();
      } else if (key === "s") {
        event.preventDefault();
        saveMask();
      } else if (key === "1") {
        setStream("raw");
      } else if (key === "2") {
        setStream("remapped");
      }
    });

    streamImg.addEventListener("load", resizeCanvas);
    window.addEventListener("resize", resizeCanvas);
    setInterval(() => {
      resizeCanvas();
      refreshStatus();
    }, 1000);
    resizeCanvas();
    refreshStatus();
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


class MaskStreamState:
    def __init__(self, name, topic, mask_path, points_path, backup_existing, jpeg_quality):
        self.name = name
        self.topic = topic
        self.mask_path = mask_path
        self.points_path = points_path
        self.backup_existing = backup_existing
        self.jpeg_quality = jpeg_quality
        self.lock = threading.Lock()
        self.condition = threading.Condition(self.lock)
        self.latest_frame = None
        self.latest_stamp = None
        self.frame_sequence = 0
        self.frame_times = []
        self.last_mask_save_path = None
        self.last_points_save_path = None
        self.last_backup_path = None
        self.running = True

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

    def status(self):
        with self.lock:
            width = self.latest_frame.shape[1] if self.latest_frame is not None else None
            height = self.latest_frame.shape[0] if self.latest_frame is not None else None
            return {
                "topic": self.topic,
                "has_frame": self.latest_frame is not None,
                "frame_sequence": self.frame_sequence,
                "fps": self.fps(),
                "width": width,
                "height": height,
                "mask_path": str(self.mask_path),
                "points_path": str(self.points_path),
                "last_mask_save_path": (
                    str(self.last_mask_save_path) if self.last_mask_save_path else None
                ),
                "last_points_save_path": (
                    str(self.last_points_save_path) if self.last_points_save_path else None
                ),
                "last_backup_path": str(self.last_backup_path) if self.last_backup_path else None,
            }

    def save_mask(self, points):
        frame, _, _ = self.snapshot()
        if frame is None:
            raise RuntimeError(f"{self.name} stream is waiting for first frame")
        if len(points) < 3:
            raise RuntimeError("Need at least 3 points before saving a mask")

        normalized_points = []
        width = frame.shape[1]
        height = frame.shape[0]
        for point in points:
            if not isinstance(point, (list, tuple)) or len(point) != 2:
                raise RuntimeError("Mask points must be [x, y] pairs")
            x = int(round(float(point[0])))
            y = int(round(float(point[1])))
            normalized_points.append(
                [max(0, min(width - 1, x)), max(0, min(height - 1, y))]
            )

        mask = build_polygon_mask(frame.shape, normalized_points)
        self.mask_path.parent.mkdir(parents=True, exist_ok=True)
        self.points_path.parent.mkdir(parents=True, exist_ok=True)

        backup_path = None
        if self.backup_existing and self.mask_path.exists():
            backup_path = backup_file_once(self.mask_path)

        if not cv2.imwrite(str(self.mask_path), mask, [cv2.IMWRITE_PNG_COMPRESSION, 3]):
            raise RuntimeError(f"Failed to write mask image: {self.mask_path}")

        points_data = {
            "stream": self.name,
            "topic": self.topic,
            "image_width": width,
            "image_height": height,
            "points": normalized_points,
        }
        with self.points_path.open("w", encoding="utf-8") as handle:
            json.dump(points_data, handle, indent=2)
            handle.write("\n")

        with self.lock:
            self.last_mask_save_path = self.mask_path
            self.last_points_save_path = self.points_path
            self.last_backup_path = backup_path

        return {
            "saved": True,
            "stream": self.name,
            "mask_path": str(self.mask_path),
            "points_path": str(self.points_path),
            "backup_path": str(backup_path) if backup_path else None,
            "image_width": width,
            "image_height": height,
            "points": normalized_points,
        }


class MaskEditorState:
    def __init__(
        self,
        raw_topic,
        remapped_topic,
        raw_mask_path,
        remapped_mask_path,
        raw_points_path,
        remapped_points_path,
        backup_existing,
        jpeg_quality,
    ):
        self.streams = {
            "raw": MaskStreamState(
                "raw",
                raw_topic,
                raw_mask_path,
                raw_points_path,
                backup_existing,
                jpeg_quality,
            ),
            "remapped": MaskStreamState(
                "remapped",
                remapped_topic,
                remapped_mask_path,
                remapped_points_path,
                backup_existing,
                jpeg_quality,
            ),
        }
        self.jpeg_quality = jpeg_quality
        self.running = True

    def stream(self, name):
        try:
            return self.streams[name]
        except KeyError as exc:
            raise RuntimeError(f"Unknown stream '{name}'") from exc

    def status(self):
        return {
            "running": self.running,
            "streams": {name: stream.status() for name, stream in self.streams.items()},
        }

    def stop(self):
        self.running = False
        for stream in self.streams.values():
            stream.running = False
            with stream.condition:
                stream.condition.notify_all()


def build_polygon_mask(image_shape, points):
    mask = np.full(image_shape[:2], 255, dtype=np.uint8)
    polygon = np.array(points, dtype=np.int32)
    cv2.fillPoly(mask, [polygon], 0)
    return mask


def backup_file_once(path):
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    candidate = path.with_name(f"{path.name}.bak.{timestamp}")
    index = 1
    while candidate.exists():
        candidate = path.with_name(f"{path.name}.bak.{timestamp}.{index}")
        index += 1
    shutil.copy2(path, candidate)
    return candidate


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
            qos_profile_sensor_data,
        )
        self.get_logger().info(f"Subscribing to remapped image topic: {self.input_topic}")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"Failed to convert image: {exc}")
            return
        self.state.update_frame(frame, msg.header.stamp)


class MaskEditorNode(Node):
    def __init__(self, state):
        super().__init__("robot_mask_editor_ui")
        self.state = state
        self.bridge = CvBridge()
        self.image_subscriptions = []
        for name, stream in state.streams.items():
            self.declare_parameter(f"{name}_topic", stream.topic)
            self.image_subscriptions.append(
                self.create_subscription(
                    Image,
                    stream.topic,
                    lambda msg, stream_name=name: self.image_callback(stream_name, msg),
                    qos_profile_sensor_data,
                )
            )
            self.get_logger().info(f"Subscribing to {name} image topic: {stream.topic}")

    def image_callback(self, stream_name, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"Failed to convert {stream_name} image: {exc}")
            return
        self.state.stream(stream_name).update_frame(frame, msg.header.stamp)


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

        def read_json(self):
            content_length = int(self.headers.get("Content-Length", "0") or "0")
            if content_length <= 0:
                return {}
            raw = self.rfile.read(content_length)
            return json.loads(raw.decode("utf-8"))

        def is_mask_editor(self):
            return hasattr(state, "streams")

        def selected_stream(self, parsed):
            if not self.is_mask_editor():
                return state
            query = parse_qs(parsed.query)
            name = query.get("stream", ["raw"])[0]
            return state.stream(name)

        def do_GET(self):
            parsed = urlparse(self.path)
            if parsed.path in ("/", "/index.html"):
                html = MASK_EDITOR_HTML if self.is_mask_editor() else INDEX_HTML
                self.send_bytes(html.encode("utf-8"), "text/html; charset=utf-8")
                return
            if parsed.path == "/status":
                if self.is_mask_editor():
                    self.send_json(state.status())
                else:
                    with state.lock:
                        payload = state.status()
                    self.send_json(payload)
                return
            if parsed.path == "/save":
                self.handle_save()
                return
            if parsed.path == "/snapshot.jpg":
                self.handle_snapshot(parsed)
                return
            if parsed.path == "/stream.mjpg":
                self.handle_stream(parsed)
                return
            self.send_error(HTTPStatus.NOT_FOUND)

        def do_POST(self):
            parsed = urlparse(self.path)
            if parsed.path == "/save":
                self.handle_save()
                return
            if parsed.path == "/mask" and self.is_mask_editor():
                self.handle_mask()
                return
            self.send_error(HTTPStatus.NOT_FOUND)

        def handle_save(self):
            if self.is_mask_editor():
                self.send_error(HTTPStatus.NOT_FOUND)
                return
            try:
                self.send_json(state.save_latest())
            except RuntimeError as exc:
                with state.lock:
                    payload = state.status(extra={"saved": False, "error": str(exc)})
                self.send_json(payload, HTTPStatus.CONFLICT)

        def handle_mask(self):
            try:
                payload = self.read_json()
                stream_name = str(payload.get("stream", "raw"))
                points = payload.get("points", [])
                self.send_json(state.stream(stream_name).save_mask(points))
            except (RuntimeError, ValueError, json.JSONDecodeError) as exc:
                self.send_json({"saved": False, "error": str(exc)}, HTTPStatus.BAD_REQUEST)

        def handle_snapshot(self, parsed):
            try:
                stream_state = self.selected_stream(parsed)
            except RuntimeError as exc:
                self.send_error(HTTPStatus.NOT_FOUND, str(exc))
                return
            frame, _, _ = stream_state.snapshot()
            if frame is None:
                self.send_error(HTTPStatus.CONFLICT, "No frame available")
                return
            ok, encoded = cv2.imencode(
                ".jpg",
                frame,
                [cv2.IMWRITE_JPEG_QUALITY, stream_state.jpeg_quality],
            )
            if not ok:
                self.send_error(HTTPStatus.INTERNAL_SERVER_ERROR, "JPEG encode failed")
                return
            self.send_bytes(encoded.tobytes(), mimetypes.types_map.get(".jpg", "image/jpeg"))

        def handle_stream(self, parsed):
            try:
                stream_state = self.selected_stream(parsed)
            except RuntimeError as exc:
                self.send_error(HTTPStatus.NOT_FOUND, str(exc))
                return
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()

            sequence = -1
            while stream_state.running:
                frame, _, sequence = stream_state.wait_for_frame_after(sequence)
                if frame is None:
                    continue
                ok, encoded = cv2.imencode(
                    ".jpg",
                    frame,
                    [cv2.IMWRITE_JPEG_QUALITY, stream_state.jpeg_quality],
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
        description="Serve a browser capture UI or web robot-mask editor for ROS image topics."
    )
    parser.add_argument(
        "--mask-editor",
        action="store_true",
        help="Serve a web polygon editor for both raw and remapped robot masks.",
    )
    parser.add_argument(
        "--input-topic",
        default="/black_feature_input_remap_node/image_remapped",
        help="Remapped sensor_msgs/Image topic to preview and save.",
    )
    parser.add_argument(
        "--raw-topic",
        default="/camera/image_raw",
        help="Raw sensor_msgs/Image topic used by --mask-editor.",
    )
    parser.add_argument(
        "--remapped-topic",
        default="/black_feature_input_remap_node/image_remapped",
        help="Remapped sensor_msgs/Image topic used by --mask-editor.",
    )
    parser.add_argument(
        "--raw-mask-output",
        default="config/mask.png",
        help="Output path for the raw-space mask in --mask-editor mode.",
    )
    parser.add_argument(
        "--remapped-mask-output",
        default="config/remapped_mask.png",
        help="Output path for the remapped-space mask in --mask-editor mode.",
    )
    parser.add_argument(
        "--raw-points-output",
        default="config/raw_mask_points.json",
        help="Output path for raw-space polygon points in --mask-editor mode.",
    )
    parser.add_argument(
        "--remapped-points-output",
        default="config/remapped_mask_points.json",
        help="Output path for remapped-space polygon points in --mask-editor mode.",
    )
    parser.add_argument(
        "--no-mask-backup",
        action="store_true",
        help="Do not create a timestamped backup before replacing an existing mask PNG.",
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


def resolve_user_path(value):
    return Path(value).expanduser().resolve()


def main():
    args = parse_args()
    jpeg_quality = min(100, max(1, args.jpeg_quality))

    if args.mask_editor:
        state = MaskEditorState(
            raw_topic=args.raw_topic,
            remapped_topic=args.remapped_topic,
            raw_mask_path=resolve_user_path(args.raw_mask_output),
            remapped_mask_path=resolve_user_path(args.remapped_mask_output),
            raw_points_path=resolve_user_path(args.raw_points_output),
            remapped_points_path=resolve_user_path(args.remapped_points_output),
            backup_existing=not args.no_mask_backup,
            jpeg_quality=jpeg_quality,
        )
    else:
        output_dir = Path(args.output_dir).expanduser().resolve()
        state = CaptureState(
            output_dir=output_dir,
            prefix=args.prefix,
            image_format=args.format,
            target_count=max(1, args.target_count),
            jpeg_quality=jpeg_quality,
        )

    rclpy.init(args=sys.argv)
    if args.mask_editor:
        node = MaskEditorNode(state)
    else:
        node = RemappedPhotoCaptureNode(state, args.input_topic)

    httpd = ThreadingHTTPServer((args.host, args.port), make_handler(state))
    httpd.daemon_threads = True

    def stop_http():
        if args.mask_editor:
            state.stop()
        else:
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
    if args.mask_editor:
        node.get_logger().info(f"Raw mask output: {state.stream('raw').mask_path}")
        node.get_logger().info(f"Remapped mask output: {state.stream('remapped').mask_path}")
    else:
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
