#!/usr/bin/env python3

from __future__ import annotations

import json
import math
import mimetypes
import re
import statistics
import threading
import time
from collections import deque
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter, parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from ultralytics import YOLO


HSV_PARAM_SPECS = [
    {"name": "black_h_min", "label": "H min", "min": 0, "max": 179, "default": 0},
    {"name": "black_h_max", "label": "H max", "min": 0, "max": 179, "default": 179},
    {"name": "black_s_min", "label": "S min", "min": 0, "max": 255, "default": 0},
    {"name": "black_s_max", "label": "S max", "min": 0, "max": 255, "default": 255},
    {"name": "black_v_min", "label": "V min", "min": 0, "max": 255, "default": 0},
    {"name": "black_v_max", "label": "V max", "min": 0, "max": 255, "default": 70},
]
HSV_PARAM_NAMES = [spec["name"] for spec in HSV_PARAM_SPECS]
HSV_PARAM_LIMITS = {
    spec["name"]: (int(spec["min"]), int(spec["max"])) for spec in HSV_PARAM_SPECS
}
DEFAULT_HSV_VALUES = {
    spec["name"]: int(spec["default"]) for spec in HSV_PARAM_SPECS
}
SAVE_LAUNCH_FILENAME = "yolo_roi_black_mask_debug.launch.py"


INDEX_HTML_TEMPLATE = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>YOLO ROI Black Mask Debug</title>
  <style>
    :root {
      color-scheme: dark;
      --bg: #0c1014;
      --panel: rgba(16, 22, 29, 0.92);
      --panel-border: rgba(129, 154, 178, 0.2);
      --panel-muted: rgba(23, 31, 40, 0.88);
      --text: #edf4fb;
      --muted: #95a6b8;
      --accent: #55c2ff;
      --accent-strong: #0091d1;
      --good: #86efac;
      --warn: #fcd34d;
      --bad: #fda4af;
      font-family: "IBM Plex Sans", "Segoe UI", sans-serif;
      background:
        radial-gradient(circle at top left, rgba(41, 121, 255, 0.18), transparent 30%),
        radial-gradient(circle at top right, rgba(255, 204, 102, 0.14), transparent 26%),
        linear-gradient(180deg, #0c1014 0%, #06090d 100%);
      color: var(--text);
    }

    * {
      box-sizing: border-box;
    }

    body {
      margin: 0;
      min-height: 100vh;
      background: transparent;
    }

    header {
      position: sticky;
      top: 0;
      z-index: 5;
      display: grid;
      grid-template-columns: minmax(0, 1fr) auto;
      gap: 16px;
      padding: 16px 20px;
      border-bottom: 1px solid var(--panel-border);
      background: rgba(7, 10, 14, 0.86);
      backdrop-filter: blur(18px);
    }

    h1 {
      margin: 0;
      font-size: 20px;
      line-height: 1.2;
      font-weight: 700;
      letter-spacing: 0.01em;
    }

    .subhead {
      margin-top: 6px;
      color: var(--muted);
      font-size: 13px;
      line-height: 1.4;
    }

    .stats {
      display: flex;
      flex-wrap: wrap;
      justify-content: flex-end;
      gap: 10px;
    }

    .stat {
      min-width: 90px;
      padding: 8px 10px;
      border: 1px solid var(--panel-border);
      border-radius: 12px;
      background: rgba(19, 26, 34, 0.88);
      box-shadow: inset 0 1px 0 rgba(255, 255, 255, 0.03);
    }

    .stat span {
      display: block;
      color: var(--muted);
      font-size: 11px;
      text-transform: uppercase;
      letter-spacing: 0.08em;
    }

    .stat strong {
      display: block;
      margin-top: 4px;
      font-size: 18px;
      font-weight: 700;
      color: var(--text);
    }

    main {
      display: grid;
      grid-template-columns: minmax(0, 1.75fr) minmax(320px, 0.95fr);
      gap: 18px;
      padding: 18px;
    }

    .panel {
      border: 1px solid var(--panel-border);
      border-radius: 18px;
      background: var(--panel);
      box-shadow: 0 18px 40px rgba(0, 0, 0, 0.25);
      overflow: hidden;
    }

    .panel-title {
      padding: 14px 16px;
      border-bottom: 1px solid var(--panel-border);
      background: rgba(255, 255, 255, 0.02);
      font-size: 13px;
      font-weight: 700;
      letter-spacing: 0.08em;
      text-transform: uppercase;
      color: var(--muted);
    }

    .viewer-grid {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 14px;
      padding: 14px;
    }

    .viewer-card {
      border: 1px solid rgba(129, 154, 178, 0.16);
      border-radius: 16px;
      background: var(--panel-muted);
      overflow: hidden;
    }

    .viewer-card figcaption {
      padding: 10px 12px;
      font-size: 13px;
      font-weight: 650;
      color: var(--text);
      background: rgba(255, 255, 255, 0.03);
      border-bottom: 1px solid rgba(129, 154, 178, 0.12);
    }

    .viewer-frame {
      display: grid;
      place-items: center;
      min-height: 280px;
      background:
        linear-gradient(135deg, rgba(52, 68, 84, 0.34), rgba(7, 10, 14, 0.92)),
        repeating-linear-gradient(
          45deg,
          rgba(255, 255, 255, 0.02) 0,
          rgba(255, 255, 255, 0.02) 12px,
          rgba(255, 255, 255, 0.01) 12px,
          rgba(255, 255, 255, 0.01) 24px
        );
    }

    img {
      display: block;
      width: 100%;
      max-height: 56vh;
      object-fit: contain;
      background: #050608;
    }

    .viewer-meta {
      padding: 10px 12px 12px;
      border-top: 1px solid rgba(129, 154, 178, 0.12);
      color: var(--muted);
      font-size: 13px;
      line-height: 1.45;
      min-height: 56px;
    }

    .control-content {
      display: grid;
      gap: 16px;
      padding: 16px;
    }

    .status-card,
    .save-card {
      padding: 14px;
      border: 1px solid rgba(129, 154, 178, 0.16);
      border-radius: 14px;
      background: rgba(18, 24, 31, 0.84);
    }

    .status-row {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 12px;
      font-size: 13px;
      color: var(--muted);
      margin-bottom: 8px;
    }

    .status-row:last-child {
      margin-bottom: 0;
    }

    .status-row strong,
    .status-row code {
      color: var(--text);
      font-weight: 650;
    }

    .status-pill {
      display: inline-flex;
      align-items: center;
      gap: 6px;
      padding: 4px 8px;
      border-radius: 999px;
      font-size: 12px;
      font-weight: 700;
    }

    .status-pill.ready {
      color: #062813;
      background: var(--good);
    }

    .status-pill.waiting {
      color: #382100;
      background: var(--warn);
    }

    .status-pill.error {
      color: #430d16;
      background: var(--bad);
    }

    .controls {
      display: grid;
      gap: 12px;
    }

    .control-row {
      padding: 12px 14px;
      border: 1px solid rgba(129, 154, 178, 0.16);
      border-radius: 14px;
      background: rgba(17, 23, 29, 0.88);
    }

    .control-head {
      display: flex;
      align-items: baseline;
      justify-content: space-between;
      gap: 12px;
      margin-bottom: 10px;
    }

    .control-head label {
      font-size: 13px;
      font-weight: 700;
      letter-spacing: 0.04em;
      text-transform: uppercase;
      color: var(--muted);
    }

    .control-value {
      font-size: 15px;
      font-weight: 700;
      color: var(--text);
    }

    input[type="range"] {
      width: 100%;
      accent-color: var(--accent);
    }

    .range-meta {
      margin-top: 6px;
      color: var(--muted);
      font-size: 12px;
    }

    button {
      appearance: none;
      border: none;
      border-radius: 12px;
      padding: 12px 14px;
      font-size: 14px;
      font-weight: 700;
      cursor: pointer;
      color: white;
      background: linear-gradient(135deg, var(--accent), var(--accent-strong));
      box-shadow: 0 10px 22px rgba(0, 145, 209, 0.26);
      transition: transform 120ms ease, filter 120ms ease;
    }

    button:hover {
      filter: brightness(1.04);
      transform: translateY(-1px);
    }

    button:disabled {
      cursor: wait;
      opacity: 0.72;
      transform: none;
    }

    .save-target,
    .save-message {
      margin-top: 10px;
      color: var(--muted);
      font-size: 12px;
      line-height: 1.45;
      word-break: break-word;
    }

    .save-message strong {
      color: var(--text);
    }

    .error-text {
      color: var(--bad);
    }

    @media (max-width: 1100px) {
      main {
        grid-template-columns: 1fr;
      }
    }

    @media (max-width: 860px) {
      header {
        grid-template-columns: 1fr;
      }

      .stats {
        justify-content: stretch;
      }

      .stat {
        flex: 1 1 92px;
      }

      .viewer-grid {
        grid-template-columns: 1fr;
      }
    }
  </style>
</head>
<body>
  <header>
    <div>
      <h1>YOLO ROI Black Mask Debug</h1>
      <div class="subhead">
        Live YOLO ROI view, exact published black mask, and in-browser HSV tuning for the running
        <code>/yolo_roi_black_mask</code> node.
      </div>
    </div>
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

  <main>
    <section class="panel">
      <div class="panel-title">Live Streams</div>
      <div class="viewer-grid">
        <figure class="viewer-card">
          <figcaption>YOLO ROI Debug Stream</figcaption>
          <div class="viewer-frame">
            <img id="yoloStream" src="/stream.mjpg?stream=yolo" alt="YOLO ROI debug stream">
          </div>
          <div class="viewer-meta" id="yoloMeta">Waiting for YOLO frames...</div>
        </figure>
        __BLACK_MASK_CARD__
      </div>
    </section>

    <aside class="panel">
      <div class="panel-title">HSV Controls</div>
      <div class="control-content">
        <div class="status-card">
          <div class="status-row">
            <span>Parameter service</span>
            <span id="serviceReady" class="status-pill waiting">Waiting</span>
          </div>
          <div class="status-row">
            <span>Apply status</span>
            <strong id="applyStatus">Waiting for initial sync</strong>
          </div>
          <div class="status-row">
            <span>Applied values</span>
            <code id="appliedValues">H 0-179 | S 0-255 | V 0-70</code>
          </div>
          <div class="status-row">
            <span>Target values</span>
            <code id="targetValues">H 0-179 | S 0-255 | V 0-70</code>
          </div>
          <div class="status-row">
            <span>Node</span>
            <code id="nodeName">/yolo_roi_black_mask</code>
          </div>
          <div class="status-row">
            <span>Last error</span>
            <strong id="lastError">None</strong>
          </div>
        </div>

        <div class="controls" id="controls"></div>

        <div class="save-card">
          <button id="saveButton" type="button">Save Launch Defaults</button>
          <div class="save-target" id="saveTarget"></div>
          <div class="save-message" id="saveMessage">Values are saved only after the latest live update has applied.</div>
        </div>
      </div>
    </aside>
  </main>

  <script>
    const HSV_FIELDS = __HSV_FIELDS__;

    const activeSlider = (name) => document.getElementById(name);
    const setText = (id, value) => { document.getElementById(id).textContent = value; };

    let pushTimer = null;
    let pushQueued = false;
    let pushInFlight = false;
    let saveInFlight = false;

    function valuesToSummary(values) {
      if (!values) return "Unavailable";
      return `H ${values.black_h_min}-${values.black_h_max} | S ${values.black_s_min}-${values.black_s_max} | V ${values.black_v_min}-${values.black_v_max}`;
    }

    function sliderValues() {
      const values = {};
      for (const field of HSV_FIELDS) {
        values[field.name] = Number(activeSlider(field.name).value);
      }
      return values;
    }

    function setSliderDisplay(name, value) {
      setText(`${name}-value`, String(value));
    }

    function syncSliders(values) {
      if (!values) return;
      for (const field of HSV_FIELDS) {
        const input = activeSlider(field.name);
        if (!input) continue;
        if (document.activeElement !== input) {
          input.value = Number(values[field.name] ?? field.default);
        }
        setSliderDisplay(field.name, input.value);
      }
    }

    function updateServiceReady(serviceReady, lastError) {
      const badge = document.getElementById("serviceReady");
      badge.className = "status-pill";
      if (serviceReady) {
        badge.classList.add("ready");
        badge.textContent = "Ready";
      } else if (lastError) {
        badge.classList.add("error");
        badge.textContent = "Error";
      } else {
        badge.classList.add("waiting");
        badge.textContent = "Waiting";
      }
    }

    function queueParamPush() {
      clearTimeout(pushTimer);
      setText("applyStatus", "Queued browser update");
      pushTimer = setTimeout(() => {
        pushQueued = true;
        void flushParamPush();
      }, 120);
    }

    async function flushParamPush() {
      if (!pushQueued || pushInFlight) {
        return;
      }
      pushQueued = false;
      pushInFlight = true;
      setText("applyStatus", "Applying live update");
      try {
        const response = await fetch("/params", {
          method: "POST",
          headers: {"Content-Type": "application/json"},
          body: JSON.stringify({values: sliderValues()}),
        });
        const data = await response.json();
        if (!response.ok) {
          throw new Error(data.error || `HTTP ${response.status}`);
        }
        syncSliders(data.target_values || {});
      } catch (error) {
        setText("applyStatus", "Browser update failed");
        setText("lastError", String(error));
      } finally {
        pushInFlight = false;
        if (pushQueued) {
          void flushParamPush();
        }
      }
    }

    async function saveDefaults() {
      if (saveInFlight) {
        return;
      }
      saveInFlight = true;
      const button = document.getElementById("saveButton");
      button.disabled = true;
      setText("saveMessage", "Saving launch defaults...");
      try {
        const response = await fetch("/save", {method: "POST"});
        const data = await response.json();
        if (!response.ok) {
          throw new Error(data.error || `HTTP ${response.status}`);
        }
        setText(
          "saveMessage",
          `Saved ${valuesToSummary(data.values)} to ${data.path}`
        );
      } catch (error) {
        setText("saveMessage", `Save failed: ${String(error)}`);
      } finally {
        saveInFlight = false;
        button.disabled = false;
      }
    }

    function updateViewerMeta(id, streamStatus, formatter) {
      const el = document.getElementById(id);
      if (!el) return;
      el.textContent = formatter(streamStatus || {});
    }

    function yoloMetaText(status) {
      if (!status.has_frame) {
        return "Waiting for YOLO frames...";
      }
      const meta = status.meta || {};
      const stats = meta.stats || {};
      return `Frame ${status.width}x${status.height} | fps ${Number(status.fps || 0).toFixed(1)} | detections ${stats.detection_count ?? 0} | max conf ${Number(stats.max_confidence ?? 0).toFixed(2)}`;
    }

    function blackMaskMetaText(status) {
      if (!status.has_frame) {
        return "Waiting for published black-mask frames...";
      }
      const meta = status.meta || {};
      const coverage = Number(meta.coverage_ratio || 0) * 100.0;
      return `Mask ${status.width}x${status.height} | fps ${Number(status.fps || 0).toFixed(1)} | nonzero ${meta.nonzero_pixels ?? 0} | coverage ${coverage.toFixed(2)}%`;
    }

    function updateStats(data) {
      const stats = data.stats || {};
      setText("detections", String(stats.detection_count ?? 0));
      setText("confidence", Number(stats.max_confidence ?? 0).toFixed(2));
      setText("rx", Number(stats.rx_fps ?? 0).toFixed(1));
      setText("proc", Number(stats.process_fps ?? 0).toFixed(1));
      setText("infer", Number(stats.infer_ms ?? 0).toFixed(1));
      setText("p95", Number(stats.p95_infer_ms ?? 0).toFixed(1));
      setText("skipped", String(stats.skipped_frames ?? 0));
    }

    function updateTuner(tuner) {
      if (!tuner) return;
      updateServiceReady(Boolean(tuner.service_ready), tuner.last_error || "");
      setText("nodeName", tuner.node_name || "/yolo_roi_black_mask");
      setText("applyStatus", tuner.apply_status || "Waiting");
      setText("appliedValues", valuesToSummary(tuner.applied_values));
      setText("targetValues", valuesToSummary(tuner.target_values));
      setText("lastError", tuner.last_error || "None");
      setText("saveTarget", tuner.save_target_path ? `Save target: ${tuner.save_target_path}` : "");
      if (tuner.last_save_result && !saveInFlight) {
        const saved = tuner.last_save_result;
        setText("saveMessage", `Last save: ${valuesToSummary(saved.values)} -> ${saved.path}`);
      }
      syncSliders(tuner.target_values);
    }

    async function refreshStatus() {
      try {
        const response = await fetch("/status", {cache: "no-store"});
        if (!response.ok) {
          return;
        }
        const data = await response.json();
        updateStats(data);
        const streams = data.streams || {};
        updateViewerMeta("yoloMeta", streams.yolo, yoloMetaText);
        updateViewerMeta("blackMaskMeta", streams.black_mask, blackMaskMetaText);
        updateTuner(data.tuner || {});
      } catch (_) {
      }
    }

    function createControls() {
      const container = document.getElementById("controls");
      for (const field of HSV_FIELDS) {
        const row = document.createElement("div");
        row.className = "control-row";
        row.innerHTML = `
          <div class="control-head">
            <label for="${field.name}">${field.label}</label>
            <span id="${field.name}-value" class="control-value">${field.default}</span>
          </div>
          <input
            id="${field.name}"
            type="range"
            min="${field.min}"
            max="${field.max}"
            step="1"
            value="${field.default}"
          >
          <div class="range-meta">Range ${field.min} to ${field.max}</div>
        `;
        container.appendChild(row);

        const input = row.querySelector("input");
        input.addEventListener("input", () => {
          setSliderDisplay(field.name, input.value);
          queueParamPush();
        });
      }
    }

    document.getElementById("saveButton").addEventListener("click", () => {
      void saveDefaults();
    });

    createControls();
    refreshStatus();
    setInterval(refreshStatus, 500);
  </script>
</body>
</html>
"""

HSV_FIELDS_JSON = json.dumps(
    [
        {
            "name": spec["name"],
            "label": spec["label"],
            "min": spec["min"],
            "max": spec["max"],
            "default": spec["default"],
        }
        for spec in HSV_PARAM_SPECS
    ]
)

BLACK_MASK_CARD_HTML = """
        <figure class="viewer-card">
          <figcaption>Published ROI Black Mask</figcaption>
          <div class="viewer-frame">
            <img id="blackMaskStream" src="/stream.mjpg?stream=black_mask" alt="Published black mask stream">
          </div>
          <div class="viewer-meta" id="blackMaskMeta">Waiting for black-mask frames...</div>
        </figure>
"""


def render_index_html(enable_black_mask_preview: bool) -> str:
    return (
        INDEX_HTML_TEMPLATE.replace("__HSV_FIELDS__", HSV_FIELDS_JSON).replace(
            "__BLACK_MASK_CARD__",
            BLACK_MASK_CARD_HTML if enable_black_mask_preview else "",
        )
    )


def resolve_model_path(model_path: str) -> Path:
    path = Path(model_path).expanduser()
    if path.is_dir():
        path = path / "weights" / "best.pt"
    return path


def clamp_box_to_image(bounds, width: int, height: int):
    if width <= 0 or height <= 0:
        return None

    x1, y1, x2, y2 = bounds
    x1 = max(0, min(width, int(x1)))
    y1 = max(0, min(height, int(y1)))
    x2 = max(0, min(width, int(x2)))
    y2 = max(0, min(height, int(y2)))
    if x2 <= x1 or y2 <= y1:
        return None
    return (x1, y1, x2, y2)


def build_roi_mask(frame_shape, detection):
    height, width = frame_shape[:2]
    roi_mask = np.zeros((height, width), dtype=np.uint8)
    if detection is None:
        return roi_mask

    bounds = clamp_box_to_image(detection["bounds"], width, height)
    if bounds is None:
        return roi_mask

    x1, y1, x2, y2 = bounds
    roi_mask[y1:y2, x1:x2] = 255
    return roi_mask


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
    index = min(
        len(ordered) - 1,
        max(0, math.ceil((pct / 100.0) * len(ordered)) - 1),
    )
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
        cv2.putText(
            frame,
            line,
            (x, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (245, 245, 245),
            1,
            cv2.LINE_AA,
        )
        y += line_height


def normalize_hsv_values(values):
    normalized = {}
    for name, value in values.items():
        if name not in HSV_PARAM_LIMITS:
            continue
        min_value, max_value = HSV_PARAM_LIMITS[name]
        normalized[name] = max(min_value, min(max_value, int(value)))
    return normalized


def resolve_save_launch_path() -> Path:
    candidates = []

    direct_source_candidate = (
        Path(__file__).resolve().parents[1] / "launch" / SAVE_LAUNCH_FILENAME
    )
    candidates.append(direct_source_candidate)

    try:
        package_prefix = Path(get_package_prefix("rcj_localization")).resolve()
        workspace_source_candidate = (
            package_prefix.parent.parent
            / "src"
            / "rcj_localization"
            / "launch"
            / SAVE_LAUNCH_FILENAME
        )
        candidates.append(workspace_source_candidate)
    except Exception:
        pass

    try:
        package_share = Path(get_package_share_directory("rcj_localization")).resolve()
        candidates.append(package_share / "launch" / SAVE_LAUNCH_FILENAME)
    except Exception:
        pass

    for candidate in candidates:
        if candidate.exists() and "/install/" not in candidate.as_posix():
            return candidate
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return direct_source_candidate


def save_launch_defaults(path: Path, values):
    text = path.read_text(encoding="utf-8")
    updated_text = text

    for name in HSV_PARAM_NAMES:
        value = str(int(values[name]))
        pattern = re.compile(
            rf'(DeclareLaunchArgument\("{re.escape(name)}", default_value=")([^"]*)(")'
        )
        updated_text, count = pattern.subn(
            lambda match, replacement=value: (
                f"{match.group(1)}{replacement}{match.group(3)}"
            ),
            updated_text,
            count=1,
        )
        if count != 1:
            raise RuntimeError(
                f"Could not find a unique default_value entry for '{name}' in {path}"
            )

    path.write_text(updated_text, encoding="utf-8")


class StreamState:
    def __init__(self, name: str, jpeg_quality: int):
        self.name = name
        self.jpeg_quality = jpeg_quality
        self.lock = threading.Lock()
        self.condition = threading.Condition(self.lock)
        self.latest_frame = None
        self.latest_meta = {}
        self.latest_sequence = 0
        self.frame_times: deque[float] = deque(maxlen=120)
        self.running = True

    def update(self, frame, meta=None):
        with self.condition:
            self.latest_frame = frame.copy()
            self.latest_meta = dict(meta or {})
            self.latest_sequence += 1
            self.frame_times.append(time.perf_counter())
            self.condition.notify_all()

    def status(self):
        with self.lock:
            width = 0
            height = 0
            if self.latest_frame is not None:
                height, width = self.latest_frame.shape[:2]
            return {
                "name": self.name,
                "has_frame": self.latest_frame is not None,
                "sequence": self.latest_sequence,
                "fps": fps_from_times(self.frame_times),
                "width": width,
                "height": height,
                "meta": dict(self.latest_meta),
            }

    def snapshot(self):
        with self.lock:
            if self.latest_frame is None:
                return None, self.latest_sequence, dict(self.latest_meta)
            return (
                self.latest_frame.copy(),
                self.latest_sequence,
                dict(self.latest_meta),
            )

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


class TunerState:
    def __init__(self, node_name: str, save_target_path: Path):
        self.lock = threading.Lock()
        self.node_name = node_name
        self.save_target_path = str(save_target_path)
        self.service_ready = False
        self.initialized = False
        self.initial_fetch_in_flight = False
        self.update_in_flight = False
        self.user_modified_target = False
        self.last_target_change_time = 0.0
        self.last_error = ""
        self.last_save_result = None
        self.target_values = dict(DEFAULT_HSV_VALUES)
        self.applied_values = dict(DEFAULT_HSV_VALUES)

    def status(self):
        with self.lock:
            if not self.service_ready:
                apply_status = "Waiting for parameter service"
            elif not self.initialized:
                apply_status = "Fetching initial HSV values"
            elif self.update_in_flight:
                apply_status = "Applying live update"
            elif self.target_values != self.applied_values:
                apply_status = "Queued browser update"
            else:
                apply_status = "Live node parameters match browser sliders"

            return {
                "node_name": self.node_name,
                "service_ready": self.service_ready,
                "initialized": self.initialized,
                "update_in_flight": self.update_in_flight,
                "target_values": dict(self.target_values),
                "applied_values": dict(self.applied_values),
                "pending_apply": self.target_values != self.applied_values,
                "last_error": self.last_error,
                "last_save_result": dict(self.last_save_result)
                if self.last_save_result
                else None,
                "save_target_path": self.save_target_path,
                "apply_status": apply_status,
            }

    def set_service_ready(self, ready: bool):
        with self.lock:
            self.service_ready = ready

    def should_fetch_initial(self) -> bool:
        with self.lock:
            return (
                self.service_ready
                and not self.initialized
                and not self.initial_fetch_in_flight
            )

    def mark_initial_fetch_started(self):
        with self.lock:
            self.initial_fetch_in_flight = True

    def apply_initial_values(self, values):
        with self.lock:
            self.initial_fetch_in_flight = False
            self.initialized = True
            self.applied_values = dict(values)
            if not self.user_modified_target:
                self.target_values = dict(values)
            self.last_error = ""

    def fail_initial_fetch(self, error: str):
        with self.lock:
            self.initial_fetch_in_flight = False
            self.last_error = error

    def update_target_values(self, values):
        normalized_values = normalize_hsv_values(values)
        with self.lock:
            for name, value in normalized_values.items():
                self.target_values[name] = value
            self.user_modified_target = True
            self.last_target_change_time = time.monotonic()
            self.last_error = ""
            return dict(self.target_values)

    def next_update_request(self, debounce_s: float):
        with self.lock:
            if not self.initialized or not self.service_ready or self.update_in_flight:
                return None
            if self.target_values == self.applied_values:
                return None
            if (time.monotonic() - self.last_target_change_time) < debounce_s:
                return None
            self.update_in_flight = True
            return dict(self.target_values)

    def complete_update(self, requested_values, success: bool, error: str = ""):
        with self.lock:
            self.update_in_flight = False
            if success:
                self.applied_values = dict(requested_values)
                self.last_error = ""
            else:
                self.last_error = error

    def values_for_save(self):
        with self.lock:
            if not self.initialized:
                raise RuntimeError("HSV parameters have not been synchronized yet.")
            if self.update_in_flight or self.target_values != self.applied_values:
                raise RuntimeError(
                    "Wait for the latest live HSV update to apply before saving."
                )
            return dict(self.applied_values)

    def record_save_result(self, path: Path, values):
        with self.lock:
            self.last_save_result = {
                "path": str(path),
                "values": dict(values),
                "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            }
            return dict(self.last_save_result)


def make_handler(streams, tuner_state: TunerState, enable_black_mask_preview: bool):
    class YoloDebugHandler(BaseHTTPRequestHandler):
        server_version = "YoloBlackCircleDebug/2.0"

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

        def selected_stream(self, parsed):
            query = parse_qs(parsed.query)
            stream_name = query.get("stream", ["yolo"])[0]
            try:
                return streams[stream_name]
            except KeyError as exc:
                raise RuntimeError(f"Unknown stream '{stream_name}'") from exc

        def status_payload(self):
            stream_statuses = {
                name: stream.status()
                for name, stream in streams.items()
            }
            yolo_meta = stream_statuses.get("yolo", {}).get("meta", {})
            return {
                "streams": stream_statuses,
                "stats": dict(yolo_meta.get("stats", {})),
                "tuner": tuner_state.status(),
            }

        def do_GET(self):
            parsed = urlparse(self.path)
            if parsed.path in ("/", "/index.html"):
                self.send_bytes(
                    render_index_html(enable_black_mask_preview).encode("utf-8"),
                    "text/html; charset=utf-8",
                )
                return
            if parsed.path == "/status":
                self.send_json(self.status_payload())
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
            if parsed.path == "/params":
                self.handle_params()
                return
            if parsed.path == "/save":
                self.handle_save()
                return
            self.send_error(HTTPStatus.NOT_FOUND)

        def handle_params(self):
            try:
                payload = self.read_json()
                values = payload.get("values", payload)
                target_values = tuner_state.update_target_values(values)
                self.send_json(
                    {
                        "ok": True,
                        "target_values": target_values,
                        "tuner": tuner_state.status(),
                    }
                )
            except (ValueError, TypeError, json.JSONDecodeError) as exc:
                self.send_json({"ok": False, "error": str(exc)}, HTTPStatus.BAD_REQUEST)

        def handle_save(self):
            try:
                values = tuner_state.values_for_save()
                path = Path(tuner_state.save_target_path)
                save_launch_defaults(path, values)
                save_result = tuner_state.record_save_result(path, values)
                self.send_json(
                    {
                        "saved": True,
                        "path": str(path),
                        "values": values,
                        "save_result": save_result,
                    }
                )
            except (RuntimeError, OSError) as exc:
                self.send_json({"saved": False, "error": str(exc)}, HTTPStatus.CONFLICT)

        def handle_snapshot(self, parsed):
            try:
                stream = self.selected_stream(parsed)
            except RuntimeError as exc:
                self.send_error(HTTPStatus.NOT_FOUND, str(exc))
                return

            frame, _, _ = stream.snapshot()
            if frame is None:
                self.send_error(HTTPStatus.CONFLICT, "No frame available")
                return

            ok, encoded = cv2.imencode(
                ".jpg",
                frame,
                [cv2.IMWRITE_JPEG_QUALITY, stream.jpeg_quality],
            )
            if not ok:
                self.send_error(HTTPStatus.INTERNAL_SERVER_ERROR, "JPEG encode failed")
                return

            self.send_bytes(
                encoded.tobytes(),
                mimetypes.types_map.get(".jpg", "image/jpeg"),
            )

        def handle_stream(self, parsed):
            try:
                stream = self.selected_stream(parsed)
            except RuntimeError as exc:
                self.send_error(HTTPStatus.NOT_FOUND, str(exc))
                return

            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()

            sequence = -1
            while stream.running:
                frame, sequence = stream.wait_for_frame_after(sequence)
                if frame is None:
                    continue
                ok, encoded = cv2.imencode(
                    ".jpg",
                    frame,
                    [cv2.IMWRITE_JPEG_QUALITY, stream.jpeg_quality],
                )
                if not ok:
                    continue
                payload = encoded.tobytes()
                try:
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(
                        f"Content-Length: {len(payload)}\r\n\r\n".encode("ascii")
                    )
                    self.wfile.write(payload)
                    self.wfile.write(b"\r\n")
                except (BrokenPipeError, ConnectionResetError, TimeoutError):
                    break

    return YoloDebugHandler


class YoloBlackCircleDebugNode(Node):
    def __init__(self):
        super().__init__("yolo_black_circle_debug")

        self.declare_parameter(
            "input_topic",
            "/black_feature_input_remap_node/image_remapped",
        )
        self.declare_parameter(
            "model_path",
            str(Path.home() / "Downloads" / "train-6" / "weights" / "best.pt"),
        )
        self.declare_parameter("confidence", 0.25)
        self.declare_parameter("iou", 0.45)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("device", "cpu")
        self.declare_parameter("max_det", 1) # Only the single most confident detection is used to define the ROI mask
        self.declare_parameter("max_processing_hz", 30.0)
        self.declare_parameter("web_host", "0.0.0.0")
        self.declare_parameter("web_port", 8081)
        self.declare_parameter("production_mode", False)
        self.declare_parameter("enable_web_viewer", True)
        self.declare_parameter("enable_yolo_overlay", True)
        self.declare_parameter("enable_black_mask_preview", True)
        self.declare_parameter("jpeg_quality", 85)
        self.declare_parameter("publish_debug_image", False)
        self.declare_parameter("debug_image_topic", "~/debug_image")
        self.declare_parameter("publish_roi_mask", True)
        self.declare_parameter("roi_mask_topic", "~/roi_mask")
        self.declare_parameter("log_interval", 30)
        self.declare_parameter("black_mask_node_name", "/yolo_roi_black_mask")
        self.declare_parameter("black_mask_topic", "/yolo_roi_black_mask/black_mask")

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.confidence = float(self.get_parameter("confidence").value)
        self.iou = float(self.get_parameter("iou").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.device = str(self.get_parameter("device").value)
        self.max_det = int(self.get_parameter("max_det").value)
        self.web_host = str(self.get_parameter("web_host").value)
        self.web_port = int(self.get_parameter("web_port").value)
        startup_override_names = set(getattr(self, "_parameter_overrides", {}).keys())
        self.production_mode = bool(self.get_parameter("production_mode").value)
        self.enable_web_viewer = self.resolve_effective_bool_parameter(
            "enable_web_viewer",
            startup_override_names,
        )
        self.enable_yolo_overlay = self.resolve_effective_bool_parameter(
            "enable_yolo_overlay",
            startup_override_names,
        )
        configured_black_mask_preview = self.resolve_effective_bool_parameter(
            "enable_black_mask_preview",
            startup_override_names,
        )
        self.enable_black_mask_preview = (
            self.enable_web_viewer and configured_black_mask_preview
        )
        self.jpeg_quality = min(100, max(1, int(self.get_parameter("jpeg_quality").value)))
        self.publish_debug_image = self.resolve_effective_bool_parameter(
            "publish_debug_image",
            startup_override_names,
        )
        self.publish_roi_mask = bool(self.get_parameter("publish_roi_mask").value)
        self.log_interval = max(1, int(self.get_parameter("log_interval").value))
        self.black_mask_node_name = str(
            self.get_parameter("black_mask_node_name").value
        )
        self.black_mask_topic = str(self.get_parameter("black_mask_topic").value)
        self.parameter_debounce_s = 0.12
        self.last_service_wait_log_time = 0.0

        max_processing_hz = float(self.get_parameter("max_processing_hz").value)
        self.model_path = resolve_model_path(str(self.get_parameter("model_path").value))
        if not self.model_path.exists():
            raise FileNotFoundError(
                f"YOLO model not found: {self.model_path}. "
                "Pass model_path:=/path/to/train-6 or /path/to/best.pt."
            )

        self.get_logger().info(f"Loading YOLO model: {self.model_path}")
        load_start = time.perf_counter()
        self.model = YOLO(str(self.model_path))
        self.get_logger().info(
            f"Loaded model in {(time.perf_counter() - load_start) * 1000.0:.1f} ms"
        )

        self.bridge = CvBridge()
        self.latest_frame = None
        self.latest_stamp = None
        self.latest_frame_id = ""
        self.latest_recv_time = 0.0
        self.latest_sequence = 0
        self.processed_sequence = 0
        self.skipped_frames = 0
        self.frame_shape = None

        self.rx_times: deque[float] = deque(maxlen=120)
        self.process_times: deque[float] = deque(maxlen=120)
        self.infer_ms: deque[float] = deque(maxlen=120)
        self.total_ms: deque[float] = deque(maxlen=120)

        self.streams = None
        self.tuner_state = None
        self.parameter_client = None
        self.web_server = None
        self.web_thread = None
        self.black_mask_subscription = None
        self.parameter_timer = None

        if self.enable_web_viewer:
            self.streams = {
                "yolo": StreamState("yolo", self.jpeg_quality),
            }
            if self.enable_black_mask_preview:
                self.streams["black_mask"] = StreamState(
                    "black_mask",
                    self.jpeg_quality,
                )
            self.tuner_state = TunerState(
                self.black_mask_node_name,
                resolve_save_launch_path(),
            )
            self.parameter_client = AsyncParameterClient(
                self,
                self.black_mask_node_name,
            )

            self.web_server = ThreadingHTTPServer(
                (self.web_host, self.web_port),
                make_handler(
                    self.streams,
                    self.tuner_state,
                    self.enable_black_mask_preview,
                ),
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
                self.get_logger().info(
                    f"Web viewer listening on http://{self.web_host}:{self.web_port}/"
                )

        self.subscription = self.create_subscription(
            Image,
            self.input_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )
        if self.enable_web_viewer and self.enable_black_mask_preview:
            self.black_mask_subscription = self.create_subscription(
                Image,
                self.black_mask_topic,
                self.black_mask_callback,
                qos_profile_sensor_data,
            )

        self.debug_publisher = None
        if self.publish_debug_image:
            self.debug_publisher = self.create_publisher(
                Image,
                str(self.get_parameter("debug_image_topic").value),
                10,
            )

        self.roi_mask_publisher = None
        if self.publish_roi_mask:
            self.roi_mask_topic = str(self.get_parameter("roi_mask_topic").value)
            self.roi_mask_publisher = self.create_publisher(
                Image,
                self.roi_mask_topic,
                10,
            )
            self.get_logger().info(f"Publishing ROI mask on {self.roi_mask_topic}")

        timer_period = 0.001
        if max_processing_hz > 0.0:
            timer_period = 1.0 / max_processing_hz
        self.timer = self.create_timer(timer_period, self.process_latest_frame)
        if self.enable_web_viewer:
            self.parameter_timer = self.create_timer(
                0.05,
                self.sync_black_mask_parameters,
            )

        self.get_logger().info(
            "production_mode=%s enable_web_viewer=%s enable_yolo_overlay=%s "
            "enable_black_mask_preview=%s publish_debug_image=%s "
            "publish_roi_mask=%s imgsz=%d device=%s max_det=%d"
            % (
                "true" if self.production_mode else "false",
                "true" if self.enable_web_viewer else "false",
                "true" if self.enable_yolo_overlay else "false",
                "true" if self.enable_black_mask_preview else "false",
                "true" if self.publish_debug_image else "false",
                "true" if self.publish_roi_mask else "false",
                self.imgsz,
                self.device,
                self.max_det,
            )
        )

    def resolve_effective_bool_parameter(self, name: str, startup_override_names) -> bool:
        value = bool(self.get_parameter(name).value)
        if (
            self.production_mode
            and name
            in {
                "enable_web_viewer",
                "enable_yolo_overlay",
                "enable_black_mask_preview",
                "publish_debug_image",
            }
            and name not in startup_override_names
        ):
            return False
        return value

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Failed to convert image: {exc}")
            return

        now = time.perf_counter()
        self.latest_frame = frame
        self.latest_stamp = msg.header.stamp
        self.latest_frame_id = msg.header.frame_id
        self.latest_recv_time = now
        self.latest_sequence += 1
        self.rx_times.append(now)

        if self.frame_shape != frame.shape:
            self.frame_shape = frame.shape
            self.get_logger().info(
                f"Receiving frames from {self.input_topic}: "
                f"{frame.shape[1]}x{frame.shape[0]}"
            )

    def black_mask_callback(self, msg: Image):
        if self.streams is None or "black_mask" not in self.streams:
            return
        try:
            mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Failed to convert black mask: {exc}")
            return

        preview = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        nonzero_pixels = int(np.count_nonzero(mask))
        coverage_ratio = 0.0
        if mask.size > 0:
            coverage_ratio = nonzero_pixels / float(mask.size)

        self.streams["black_mask"].update(
            preview,
            {
                "nonzero_pixels": nonzero_pixels,
                "coverage_ratio": coverage_ratio,
                "frame_id": msg.header.frame_id,
            },
        )

    def sync_black_mask_parameters(self):
        if self.parameter_client is None or self.tuner_state is None:
            return
        services_ready = self.parameter_client.services_are_ready()
        self.tuner_state.set_service_ready(services_ready)

        if not services_ready:
            now = time.monotonic()
            if now - self.last_service_wait_log_time >= 2.0:
                self.last_service_wait_log_time = now
                self.get_logger().info(
                    f"Waiting for parameter services from {self.black_mask_node_name}"
                )
            return

        if self.tuner_state.should_fetch_initial():
            self.tuner_state.mark_initial_fetch_started()
            future = self.parameter_client.get_parameters(HSV_PARAM_NAMES)
            future.add_done_callback(self.handle_initial_parameter_result)
            return

        requested_values = self.tuner_state.next_update_request(self.parameter_debounce_s)
        if requested_values is None:
            return

        parameters = [
            Parameter(name, Parameter.Type.INTEGER, int(requested_values[name]))
            for name in HSV_PARAM_NAMES
        ]
        future = self.parameter_client.set_parameters(parameters)
        future.add_done_callback(
            lambda completed_future, values=requested_values: (
                self.handle_parameter_update_result(completed_future, values)
            )
        )

    def handle_initial_parameter_result(self, future):
        if self.tuner_state is None:
            return
        try:
            response = future.result()
            values = {}
            for name, parameter_value in zip(HSV_PARAM_NAMES, response.values):
                python_value = parameter_value_to_python(parameter_value)
                if python_value is None:
                    raise RuntimeError(f"Parameter '{name}' is not set on the target node.")
                values[name] = int(python_value)
            self.tuner_state.apply_initial_values(normalize_hsv_values(values))
        except Exception as exc:  # noqa: BLE001
            self.tuner_state.fail_initial_fetch(str(exc))

    def handle_parameter_update_result(self, future, requested_values):
        if self.tuner_state is None:
            return
        try:
            response = future.result()
            failed_reasons = [
                result.reason or "unknown reason"
                for result in response.results
                if not result.successful
            ]
            if failed_reasons:
                self.tuner_state.complete_update(
                    requested_values,
                    False,
                    "; ".join(failed_reasons),
                )
                return
            self.tuner_state.complete_update(requested_values, True)
        except Exception as exc:  # noqa: BLE001
            self.tuner_state.complete_update(requested_values, False, str(exc))

    def process_latest_frame(self):
        if self.latest_frame is None:
            return
        if self.latest_sequence == self.processed_sequence:
            return

        sequence = self.latest_sequence
        skipped_now = max(0, sequence - self.processed_sequence - 1)
        self.skipped_frames += skipped_now
        frame = self.latest_frame.copy()
        frame_stamp = self.latest_stamp
        frame_id = self.latest_frame_id
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
        detections = self.extract_detections(result)
        selected_detection = max(
            detections,
            key=lambda det: det["confidence"],
            default=None,
        )
        roi_mask = build_roi_mask(frame.shape, selected_detection)
        total_elapsed_ms = (time.perf_counter() - total_start) * 1000.0

        now = time.perf_counter()
        self.process_times.append(now)
        self.infer_ms.append(infer_elapsed_ms)
        self.total_ms.append(total_elapsed_ms)

        max_conf = max((det["confidence"] for det in detections), default=0.0)
        stats = self.make_stats(len(detections), max_conf, now - recv_time)
        overlay_needed = (
            self.enable_yolo_overlay
            or self.debug_publisher is not None
            or self.enable_web_viewer
        )
        output_frame = frame
        if overlay_needed:
            output_frame = frame.copy()
            self.draw_detections(output_frame, detections)
            self.draw_stats(output_frame, stats)

        if self.enable_web_viewer and self.streams is not None:
            self.streams["yolo"].update(output_frame, {"stats": stats})

        if self.debug_publisher is not None:
            msg = self.bridge.cv2_to_imgmsg(output_frame, encoding="bgr8")
            if frame_stamp is not None:
                msg.header.stamp = frame_stamp
            msg.header.frame_id = "yolo_black_circle_debug"
            self.debug_publisher.publish(msg)

        if self.roi_mask_publisher is not None:
            msg = self.bridge.cv2_to_imgmsg(roi_mask, encoding="mono8")
            if frame_stamp is not None:
                msg.header.stamp = frame_stamp
            msg.header.frame_id = frame_id
            self.roi_mask_publisher.publish(msg)

        if len(self.process_times) % self.log_interval == 0:
            self.log_stats(sequence, len(detections), max_conf, skipped_now)

    def extract_detections(self, result):
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
            label = (
                names.get(class_id, str(class_id))
                if isinstance(names, dict)
                else str(class_id)
            )
            detections.append(
                {
                    "bounds": (x1, y1, x2, y2),
                    "confidence": float(confidence),
                    "class_id": int(class_id),
                    "label": label,
                }
            )
        return detections

    def draw_detections(self, frame, detections):
        for detection in detections:
            x1, y1, x2, y2 = detection["bounds"]
            confidence = detection["confidence"]
            label = detection["label"]
            color = (0, 255, 120)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            center = ((x1 + x2) // 2, (y1 + y2) // 2)
            radius = max(2, min(x2 - x1, y2 - y1) // 2)
            cv2.circle(frame, center, radius, (0, 180, 255), 1)
            text = f"{label} {confidence:.2f}"
            text_size, baseline = cv2.getTextSize(
                text,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                1,
            )
            text_y = max(18, y1 - 7)
            cv2.rectangle(
                frame,
                (x1, text_y - text_size[1] - baseline - 4),
                (x1 + text_size[0] + 6, text_y + baseline),
                (0, 90, 55),
                -1,
            )
            cv2.putText(
                frame,
                text,
                (x1 + 3, text_y - 3),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

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
        if self.streams is not None:
            for stream in self.streams.values():
                stream.stop()
        if self.web_server is not None:
            self.web_server.shutdown()
            self.web_server.server_close()
        if self.web_thread is not None:
            self.web_thread.join(timeout=1.0)
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
