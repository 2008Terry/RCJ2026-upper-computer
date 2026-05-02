#!/usr/bin/env python3

from __future__ import annotations

import json
import math
import mimetypes
import threading
import time
from dataclasses import dataclass
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, urlparse

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseArray, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image, PointCloud2
from std_msgs.msg import Float32


INDEX_HTML = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>RCJ Localization Debug</title>
  <style>
    :root {
      color-scheme: dark;
      font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      background: #0f1216;
      color: #f2f5f8;
    }
    * { box-sizing: border-box; }
    body { margin: 0; min-height: 100vh; background: #0f1216; }
    button, input, select { font: inherit; }
    header {
      display: grid;
      grid-template-columns: minmax(180px, 1fr) auto;
      gap: 12px;
      align-items: center;
      padding: 10px 14px;
      border-bottom: 1px solid #2d333b;
      background: #171b21;
      position: sticky;
      top: 0;
      z-index: 5;
    }
    h1 { margin: 0; font-size: 18px; line-height: 1.2; letter-spacing: 0; }
    .status { display: flex; flex-wrap: wrap; justify-content: flex-end; gap: 8px; }
    .pill {
      border: 1px solid #343c45;
      background: #20262e;
      border-radius: 8px;
      padding: 5px 8px;
      min-width: 84px;
    }
    .pill span { display: block; color: #a9b4bf; font-size: 11px; line-height: 1.1; }
    .pill strong { display: block; color: #f8fafc; font-size: 15px; line-height: 1.2; }
    main {
      display: grid;
      grid-template-columns: minmax(280px, 360px) minmax(0, 1fr);
      min-height: calc(100vh - 58px);
    }
    aside {
      border-right: 1px solid #2d333b;
      background: #14181e;
      overflow: auto;
      max-height: calc(100vh - 58px);
    }
    section { padding: 12px; }
    h2 { margin: 10px 0 8px; font-size: 14px; line-height: 1.2; letter-spacing: 0; }
    .stream-list {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 6px;
    }
    .stream-btn, .action-btn {
      min-height: 34px;
      border: 1px solid #37414c;
      border-radius: 8px;
      background: #202730;
      color: #edf2f7;
      padding: 6px 8px;
      text-align: left;
      cursor: pointer;
    }
    .stream-btn.active { border-color: #56b6c2; background: #21343a; }
    .stream-btn small { display: block; color: #a8b3bd; font-size: 10px; line-height: 1.1; }
    .viewer {
      display: grid;
      grid-template-rows: minmax(280px, 1fr) auto;
      gap: 10px;
      min-height: calc(100vh - 82px);
    }
    .image-wrap {
      display: grid;
      place-items: center;
      min-height: 0;
      background: #07090b;
      border: 1px solid #29313a;
      border-radius: 8px;
      overflow: hidden;
    }
    #stream {
      display: block;
      max-width: 100%;
      max-height: calc(100vh - 210px);
      object-fit: contain;
    }
    .map-wrap {
      display: grid;
      grid-template-columns: minmax(0, 1fr) 210px;
      gap: 10px;
    }
    .map-wrap.disabled {
      display: none;
    }
    canvas {
      width: 100%;
      min-height: 260px;
      border: 1px solid #29313a;
      border-radius: 8px;
      background: #060708;
    }
    .panel {
      border-top: 1px solid #2d333b;
      padding: 10px 12px 14px;
    }
    .control {
      display: grid;
      grid-template-columns: minmax(96px, 0.75fr) minmax(110px, 1fr) 74px;
      align-items: center;
      gap: 8px;
      margin: 7px 0;
    }
    .control label { color: #cbd5df; font-size: 12px; }
    input[type="range"] { width: 100%; }
    input[type="number"] {
      width: 74px;
      border: 1px solid #3a4551;
      border-radius: 8px;
      padding: 5px 6px;
      background: #0d1014;
      color: #f8fafc;
    }
    .toggle {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 8px;
      min-height: 30px;
      color: #cbd5df;
      font-size: 12px;
    }
    .action-row { display: flex; flex-wrap: wrap; gap: 8px; margin-top: 10px; }
    .action-btn { text-align: center; min-height: 32px; }
    pre {
      white-space: pre-wrap;
      overflow-wrap: anywhere;
      min-height: 88px;
      margin: 8px 0 0;
      border: 1px solid #2f3842;
      border-radius: 8px;
      background: #0b0e12;
      color: #d6dee7;
      padding: 8px;
      font-size: 12px;
      line-height: 1.35;
    }
    .notice { color: #aab4bf; font-size: 12px; line-height: 1.35; margin: 6px 0 0; }
    .error { color: #ffb4a8; }
    .boot-error {
      margin: 10px 12px 0;
      padding: 8px 10px;
      border: 1px solid #7f3b34;
      border-radius: 8px;
      background: #2a1514;
      color: #ffb4a8;
      display: none;
      white-space: pre-wrap;
    }
    .empty-view {
      color: #8d98a5;
      font-size: 14px;
      text-align: center;
      padding: 24px;
    }
    @media (max-width: 900px) {
      header { grid-template-columns: 1fr; }
      .status { justify-content: stretch; }
      .pill { flex: 1 1 92px; }
      main { grid-template-columns: 1fr; }
      aside { max-height: none; border-right: 0; border-bottom: 1px solid #2d333b; }
      .map-wrap { grid-template-columns: 1fr; }
      #stream { max-height: 58vh; }
    }
  </style>
</head>
<body>
  <div id="bootError" class="boot-error"></div>
  <header>
    <h1>RCJ Localization Debug</h1>
    <div class="status">
      <div class="pill"><span>Stream</span><strong id="activeStream">-</strong></div>
      <div class="pill"><span>Image Age</span><strong id="imageAge">-</strong></div>
      <div class="pill"><span>RViz Loc</span><strong id="rvizState">-</strong></div>
      <div class="pill"><span>Pose</span><strong id="poseAge">-</strong></div>
      <div class="pill"><span>Particles</span><strong id="particles">0</strong></div>
      <div class="pill"><span>Proc ms</span><strong id="processing">-</strong></div>
    </div>
  </header>
  <main>
    <aside>
      <section>
        <h2>Streams</h2>
        <div id="streams" class="stream-list"></div>
        <p class="notice">Streams use existing ROS debug topics. Enable publish toggles below if a view is stale.</p>
      </section>
      <section class="panel">
        <h2>RViz Localization</h2>
        <label class="toggle">
          <span>Display map, pose, particles</span>
          <input id="rvizDisplayToggle" type="checkbox">
        </label>
        <p class="notice">This state is saved on the robot. Turning it off reduces dashboard status payload size over Wi-Fi.</p>
      </section>
      <section class="panel">
        <h2>HSV Thresholds</h2>
        <div id="hsvControls"></div>
        <div class="action-row">
          <button class="action-btn" id="applyHsv">Apply HSV</button>
          <button class="action-btn" id="refreshParams">Refresh</button>
        </div>
      </section>
      <section class="panel">
        <h2>Debug Toggles</h2>
        <div id="toggleControls"></div>
        <div class="action-row">
          <button class="action-btn" id="applyToggles">Apply Toggles</button>
        </div>
      </section>
      <section class="panel">
        <h2>Localization Tuning</h2>
        <div id="locControls"></div>
        <div class="action-row">
          <button class="action-btn" id="applyLoc">Apply Localization</button>
        </div>
      </section>
      <section class="panel">
        <h2>Export</h2>
        <button class="action-btn" id="buildExport">Build Launch Args</button>
        <pre id="exportText"></pre>
        <p class="notice" id="message"></p>
      </section>
    </aside>
    <section class="viewer">
      <div class="image-wrap">
        <div id="streamPlaceholder" class="empty-view">Loading dashboard...</div>
        <img id="stream" alt="Selected debug stream">
      </div>
      <div class="map-wrap" id="localizationPanel">
        <canvas id="map" width="900" height="560"></canvas>
        <pre id="localizationText"></pre>
      </div>
    </section>
  </main>
  <script>
    window.onerror = function(message, source, line, column) {
      var target = document.getElementById("bootError");
      if (!target) return false;
      target.style.display = "block";
      target.textContent = "Dashboard script error: " + message + " at " + line + ":" + column;
      return false;
    };
  </script>
  <script>
    const HSV = [
      ["white_h_min", 0, 179], ["white_h_max", 0, 179], ["white_s_max", 0, 255],
      ["white_v_min", 0, 255], ["black_v_max", 0, 255],
      ["green_h_min", 0, 179], ["green_h_max", 0, 179],
      ["green_s_min", 0, 255], ["green_v_min", 0, 255]
    ];
    const LOC = [
      ["num_particles", "int"], ["sigma_hit", "float"], ["noise_xy", "float"],
      ["noise_theta", "float"], ["random_injection_max_ratio", "float"],
      ["occupancy_threshold", "int"], ["distance_transform_mask_size", "int"],
      ["init_field_width", "float"], ["init_field_height", "float"], ["filter_period_ms", "int"]
    ];
    const TOGGLES = [
      ["hsv", "publish_debug_images"], ["hsv", "publish_input_image"], ["hsv", "publish_white_mask"],
      ["hsv", "publish_green_mask"], ["hsv", "publish_black_mask"], ["hsv", "publish_noise_mask"],
      ["hsv", "publish_overlay_image"], ["remap", "publish_debug_images"],
      ["ridge", "publish_debug_images"], ["ridge", "publish_debug_image"],
      ["localization", "publish_debug_pointcloud"]
    ];
    const state = {
      streams: {},
      active: "hsv_overlay",
      params: {},
      localization: {},
      display: {localization_visible: true}
    };
    function $(id) { return document.getElementById(id); }
    function objectValues(obj) {
      return Object.keys(obj || {}).map(function(key) { return obj[key]; });
    }
    function valueOr(value, fallback) {
      return value === undefined || value === null ? fallback : value;
    }
    function paramValue(node, name) {
      return state.params[node] ? state.params[node][name] : undefined;
    }
    const age = function(stamp) {
      if (!stamp) return "-";
      const seconds = Math.max(0, (Date.now() / 1000) - stamp);
      return seconds < 10 ? seconds.toFixed(1) + "s" : Math.round(seconds) + "s";
    };
    const setMessage = function(text, isError=false) {
      $("message").textContent = text || "";
      $("message").className = isError ? "notice error" : "notice";
    };
    function streamLabel(stream) {
      return stream.label || stream.id;
    }
    function renderStreams() {
      const root = $("streams");
      root.innerHTML = "";
      objectValues(state.streams).forEach(function(stream) {
        const btn = document.createElement("button");
        btn.className = "stream-btn" + (state.active === stream.id ? " active" : "");
        const subscribed = stream.subscribed ? "subscribed" : "idle";
        btn.textContent = streamLabel(stream);
        const detail = document.createElement("small");
        detail.textContent = (stream.status || "waiting") + " / " + subscribed;
        btn.appendChild(detail);
        btn.onclick = function() { selectStream(stream.id); };
        root.appendChild(btn);
      });
    }
    async function selectStream(id) {
      state.active = id;
      $("activeStream").textContent = streamLabel(state.streams[id] || {id});
      renderStreams();
      try {
        const data = await getJson("/api/stream", {
          method: "POST",
          headers: {"Content-Type": "application/json"},
          body: JSON.stringify({stream_id: id})
        });
        state.active = data.active_stream || id;
        if (data.warnings && data.warnings.length) {
          setMessage(data.warnings.join("\\n"), true);
        }
      } catch (err) {
        setMessage(String(err), true);
      }
      $("streamPlaceholder").textContent = "Waiting for " + streamLabel(state.streams[state.active] || {id: state.active}) + "...";
      $("streamPlaceholder").style.display = "block";
      $("stream").style.display = "none";
      $("stream").src = "/stream/" + encodeURIComponent(state.active) + ".mjpg?t=" + Date.now();
      $("activeStream").textContent = streamLabel(state.streams[state.active] || {id: state.active});
      renderStreams();
    }
    function makeRange(root, spec, value) {
      const [name, min, max] = spec;
      const row = document.createElement("div");
      row.className = "control";
      const label = document.createElement("label");
      label.textContent = name;
      const wrap = document.createElement("div");
      const range = document.createElement("input");
      range.type = "range"; range.min = min; range.max = max; range.value = valueOr(value, min);
      const num = document.createElement("input");
      num.type = "number"; num.min = min; num.max = max; num.value = range.value;
      range.dataset.name = name; num.dataset.name = name;
      range.oninput = function() { num.value = range.value; };
      num.oninput = function() { range.value = num.value; };
      wrap.appendChild(range);
      row.appendChild(label); row.appendChild(wrap); row.appendChild(num); root.appendChild(row);
    }
    function makeNumber(root, node, name, type, value) {
      const row = document.createElement("div");
      row.className = "control";
      const label = document.createElement("label");
      label.textContent = name;
      const num = document.createElement("input");
      num.type = "number";
      num.step = type === "int" ? "1" : "0.001";
      num.value = valueOr(value, 0);
      num.dataset.node = node; num.dataset.name = name; num.dataset.type = type;
      row.appendChild(label); row.appendChild(num); root.appendChild(row);
    }
    function renderControls() {
      const hsv = $("hsvControls");
      hsv.innerHTML = "";
      HSV.forEach(function(spec) { makeRange(hsv, spec, paramValue("hsv", spec[0])); });
      const loc = $("locControls");
      loc.innerHTML = "";
      LOC.forEach(function(item) { makeNumber(loc, "localization", item[0], item[1], paramValue("localization", item[0])); });
      const toggles = $("toggleControls");
      toggles.innerHTML = "";
      TOGGLES.forEach(function(item) {
        const node = item[0];
        const name = item[1];
        const row = document.createElement("label");
        row.className = "toggle";
        const input = document.createElement("input");
        input.type = "checkbox"; input.dataset.node = node; input.dataset.name = name;
        input.checked = Boolean(paramValue(node, name));
        row.appendChild(document.createTextNode(node + "." + name));
        row.appendChild(input);
        toggles.appendChild(row);
      });
    }
    function renderDisplayState() {
      const visible = state.display.localization_visible !== false;
      $("rvizState").textContent = visible ? "On" : "Off";
      $("rvizDisplayToggle").checked = visible;
      $("localizationPanel").classList.toggle("disabled", !visible);
      if (!visible) {
        $("poseAge").textContent = "Off";
        $("particles").textContent = "Off";
        $("processing").textContent = "Off";
      }
    }
    async function getJson(url, options) {
      const res = await fetch(url, options);
      const data = await res.json();
      if (!res.ok || data.ok === false) throw new Error(data.error || data.reason || res.statusText);
      return data;
    }
    async function refreshStatus() {
      try {
        const data = await getJson("/api/status", {cache: "no-store"});
        state.streams = data.streams || {};
        state.active = data.active_stream || state.active;
        state.display = data.display || state.display;
        state.localization = data.localization || {};
        const active = state.streams[state.active] || objectValues(state.streams)[0];
        if (active && (!$("stream").src || !state.streams[state.active])) selectStream(active.id);
        $("imageAge").textContent = active ? age(active.updated_at) : "-";
        renderDisplayState();
        if (state.display.localization_visible !== false) {
          $("poseAge").textContent = age(state.localization.pose ? state.localization.pose.updated_at : null);
          $("particles").textContent = String(valueOr(state.localization.particle_count, 0));
          $("processing").textContent = state.localization.processing_time_ms == null ? "-" : Number(state.localization.processing_time_ms).toFixed(1);
        }
        renderStreams();
        renderLocalization();
      } catch (err) {
        setMessage(String(err), true);
      }
    }
    async function refreshParams() {
      const data = await getJson("/api/parameters", {cache: "no-store"});
      state.params = data.parameters || {};
      renderControls();
    }
    async function postParams(node, params) {
      const data = await getJson("/api/parameters", {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify({node, parameters: params})
      });
      setMessage(data.message || "Updated.");
      await refreshParams();
    }
    function collectHsv() {
      const values = {};
      $("hsvControls").querySelectorAll("input[type='range']").forEach(function(input) {
        values[input.dataset.name] = Number(input.value);
      });
      return values;
    }
    function collectLoc() {
      const values = {};
      $("locControls").querySelectorAll("input").forEach(function(input) {
        values[input.dataset.name] = input.dataset.type === "int" ? parseInt(input.value, 10) : Number(input.value);
      });
      return values;
    }
    function collectToggles() {
      const grouped = {};
      $("toggleControls").querySelectorAll("input").forEach(function(input) {
        if (!grouped[input.dataset.node]) grouped[input.dataset.node] = {};
        grouped[input.dataset.node][input.dataset.name] = input.checked;
      });
      return grouped;
    }
    async function applyToggles() {
      const grouped = collectToggles();
      const failures = [];
      for (const [node, parameters] of Object.entries(grouped)) {
        try { await postParams(node, parameters); } catch (err) { failures.push(node + ": " + err); }
      }
      if (failures.length) setMessage(failures.join("\\n"), true);
    }
    async function setRvizDisplay(enabled) {
      const data = await getJson("/api/display", {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify({localization_visible: enabled})
      });
      state.display = data.display || state.display;
      renderDisplayState();
      renderLocalization();
      setMessage("Saved RViz localization display: " + (enabled ? "on" : "off"));
    }
    function renderLocalization() {
      if (state.display.localization_visible === false) {
        $("localizationText").textContent = "RViz localization display is off.";
        return;
      }
      const loc = state.localization || {};
      const pose = loc.pose || {};
      $("localizationText").textContent =
        "pose: x=" + fmt(pose.x) + " y=" + fmt(pose.y) + " yaw=" + fmt(pose.yaw_deg) + " deg\\n" +
        "particles: " + valueOr(loc.particle_count, 0) + "\\n" +
        "debug pointcloud: " + valueOr(loc.pointcloud_points, 0) + " points\\n" +
        "processing: " + (loc.processing_time_ms == null ? "-" : fmt(loc.processing_time_ms)) + " ms";
      drawMap(loc);
    }
    function fmt(v) {
      return v == null || !Number.isFinite(Number(v)) ? "-" : Number(v).toFixed(3);
    }
    function drawMap(loc) {
      const canvas = $("map");
      const ctx = canvas.getContext("2d");
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.fillStyle = "#060708";
      ctx.fillRect(0, 0, canvas.width, canvas.height);
      const map = loc.map;
      if (!map || !map.width || !map.height) {
        ctx.fillStyle = "#9aa6b2";
        ctx.fillText("Waiting for /map", 16, 24);
        return;
      }
      const scale = Math.min(canvas.width / map.width, canvas.height / map.height);
      const ox = (canvas.width - map.width * scale) / 2;
      const oy = (canvas.height - map.height * scale) / 2;
      const data = map.preview || [];
      if (data.length) {
        const img = ctx.createImageData(map.width, map.height);
        for (let i = 0; i < data.length; i++) {
          const v = data[i];
          const c = v < 0 ? 45 : (255 - Math.min(100, Math.max(0, v)) * 2.25);
          const p = i * 4;
          img.data[p] = c; img.data[p+1] = c; img.data[p+2] = c; img.data[p+3] = 255;
        }
        const off = document.createElement("canvas");
        off.width = map.width; off.height = map.height;
        off.getContext("2d").putImageData(img, 0, 0);
        ctx.imageSmoothingEnabled = false;
        ctx.drawImage(off, ox, oy, map.width * scale, map.height * scale);
      }
      function worldToCanvas(x, y) {
        const mx = (x - map.origin_x) / map.resolution;
        const my = (y - map.origin_y) / map.resolution;
        return [ox + mx * scale, oy + (map.height - my) * scale];
      }
      ctx.fillStyle = "rgba(86, 182, 194, 0.45)";
      (loc.particles || []).forEach(function(p) {
        const [x, y] = worldToCanvas(p.x, p.y);
        ctx.fillRect(x - 1.5, y - 1.5, 3, 3);
      });
      if (loc.pose) {
        const [x, y] = worldToCanvas(loc.pose.x, loc.pose.y);
        const yaw = -loc.pose.yaw;
        ctx.save();
        ctx.translate(x, y);
        ctx.rotate(yaw);
        ctx.fillStyle = "#ffcc66";
        ctx.beginPath();
        ctx.moveTo(10, 0); ctx.lineTo(-7, -6); ctx.lineTo(-4, 0); ctx.lineTo(-7, 6);
        ctx.closePath(); ctx.fill();
        ctx.restore();
      }
    }
    async function buildExport() {
      const data = await getJson("/api/export", {cache: "no-store"});
      $("exportText").textContent = data.launch_arguments || "";
    }
    $("applyHsv").onclick = function() { postParams("hsv", collectHsv()).catch(function(e) { setMessage(String(e), true); }); };
    $("applyLoc").onclick = function() { postParams("localization", collectLoc()).catch(function(e) { setMessage(String(e), true); }); };
    $("applyToggles").onclick = function() { applyToggles(); };
    $("refreshParams").onclick = function() { refreshParams().catch(function(e) { setMessage(String(e), true); }); };
    $("buildExport").onclick = function() { buildExport().catch(function(e) { setMessage(String(e), true); }); };
    $("stream").onload = function() {
      $("streamPlaceholder").style.display = "none";
      $("stream").style.display = "block";
    };
    $("stream").onerror = function() {
      $("streamPlaceholder").style.display = "block";
      $("stream").style.display = "none";
      $("streamPlaceholder").textContent = "Stream request failed for " + streamLabel(state.streams[state.active] || {id: state.active}) + ".";
    };
    $("rvizDisplayToggle").onchange = function(event) {
      setRvizDisplay(event.target.checked).catch(function(e) {
        event.target.checked = !event.target.checked;
        setMessage(String(e), true);
      });
    };
    selectStream(state.active);
    refreshParams().catch(function() { renderControls(); });
    refreshStatus();
    setInterval(refreshStatus, 750);
  </script>
</body>
</html>
"""


@dataclass(frozen=True)
class StreamConfig:
    stream_id: str
    label: str
    compressed_topic: str
    raw_topic: str


STREAMS = [
    StreamConfig("camera_raw", "Camera Raw", "/camera/image_raw/compressed_debug", "/camera/image_raw"),
    StreamConfig("remap_input", "Remap Input", "/white_line_hsv_input_remap_node/debug/input_image/compressed", "/white_line_hsv_input_remap_node/debug/input_image"),
    StreamConfig("remap_output", "Remap Output", "/white_line_hsv_input_remap_node/debug/output_image/compressed", "/white_line_hsv_input_remap_node/debug/output_image"),
    StreamConfig("hsv_input", "HSV Input", "/white_line_hsv_white_node/debug/input_image/compressed", "/white_line_hsv_white_node/debug/input_image"),
    StreamConfig("hsv_white", "HSV White", "/white_line_hsv_white_node/debug/white_mask/compressed", "/white_line_hsv_white_node/debug/white_mask"),
    StreamConfig("hsv_green", "HSV Green", "/white_line_hsv_white_node/debug/green_mask/compressed", "/white_line_hsv_white_node/debug/green_mask"),
    StreamConfig("hsv_black", "HSV Black", "/white_line_hsv_white_node/debug/black_mask/compressed", "/white_line_hsv_white_node/debug/black_mask"),
    StreamConfig("hsv_noise", "HSV Noise", "/white_line_hsv_white_node/debug/noise_mask/compressed", "/white_line_hsv_white_node/debug/noise_mask"),
    StreamConfig("hsv_overlay", "HSV Overlay", "/white_line_hsv_white_node/debug/overlay_image/compressed", "/white_line_hsv_white_node/debug/overlay_image"),
    StreamConfig("ridge_debug", "Ridge Debug", "/white_line_dt_ridge_filter_node/debug_image/compressed", "/white_line_dt_ridge_filter_node/debug_image"),
    StreamConfig("ridge_mask", "Ridge Mask", "/white_line_dt_ridge_filter_node/ridge_mask/compressed", "/white_line_dt_ridge_filter_node/ridge_mask"),
    StreamConfig("ridge_orientation", "Orientation", "/white_line_dt_ridge_filter_node/orientation_valid_mask/compressed", "/white_line_dt_ridge_filter_node/orientation_valid_mask"),
    StreamConfig("ridge_side", "Side Support", "/white_line_dt_ridge_filter_node/side_support_mask/compressed", "/white_line_dt_ridge_filter_node/side_support_mask"),
    StreamConfig("ridge_reconstructed", "Reconstructed", "/white_line_dt_ridge_filter_node/reconstructed_mask/compressed", "/white_line_dt_ridge_filter_node/reconstructed_mask"),
    StreamConfig("ridge_final", "Final White", "/white_line_dt_ridge_filter_node/white_final_mask/compressed", "/white_line_dt_ridge_filter_node/white_final_mask"),
]

PARAM_NODES = {
    "hsv": "/white_line_hsv_white_node",
    "remap": "/white_line_hsv_input_remap_node",
    "ridge": "/white_line_dt_ridge_filter_node",
    "localization": "/amcl_fusion",
}

PARAMETER_NAMES = {
    "hsv": [
        "white_h_min", "white_h_max", "white_s_max", "white_v_min",
        "black_v_max", "green_h_min", "green_h_max", "green_s_min", "green_v_min",
        "publish_debug_images", "publish_input_image", "publish_white_mask",
        "publish_green_mask", "publish_black_mask", "publish_noise_mask", "publish_overlay_image",
    ],
    "remap": ["publish_debug_images", "publish_input_image", "publish_output_image"],
    "ridge": [
        "publish_debug_images", "publish_debug_image", "publish_ridge_mask",
        "publish_orientation_valid_mask", "publish_side_support_mask",
        "publish_reconstructed_mask", "publish_white_final_mask",
    ],
    "localization": [
        "num_particles", "sigma_hit", "noise_xy", "noise_theta",
        "random_injection_max_ratio", "occupancy_threshold", "distance_transform_mask_size",
        "init_field_width", "init_field_height", "filter_period_ms", "publish_debug_pointcloud",
    ],
}

STREAM_ENABLE_PARAMS = {
    "remap_input": ("remap", {"publish_debug_images": True, "publish_input_image": True}),
    "remap_output": ("remap", {"publish_debug_images": True, "publish_output_image": True}),
    "hsv_input": ("hsv", {"publish_debug_images": True, "publish_input_image": True}),
    "hsv_white": ("hsv", {"publish_debug_images": True, "publish_white_mask": True}),
    "hsv_green": ("hsv", {"publish_debug_images": True, "publish_green_mask": True}),
    "hsv_black": ("hsv", {"publish_debug_images": True, "publish_black_mask": True}),
    "hsv_noise": ("hsv", {"publish_debug_images": True, "publish_noise_mask": True}),
    "hsv_overlay": ("hsv", {"publish_debug_images": True, "publish_overlay_image": True}),
    "ridge_debug": ("ridge", {"publish_debug_images": True, "publish_debug_image": True}),
    "ridge_mask": ("ridge", {"publish_debug_images": True, "publish_ridge_mask": True}),
    "ridge_orientation": (
        "ridge",
        {"publish_debug_images": True, "publish_orientation_valid_mask": True},
    ),
    "ridge_side": ("ridge", {"publish_debug_images": True, "publish_side_support_mask": True}),
    "ridge_reconstructed": (
        "ridge",
        {"publish_debug_images": True, "publish_reconstructed_mask": True},
    ),
    "ridge_final": ("ridge", {"publish_debug_images": True, "publish_white_final_mask": True}),
}


class StreamState:
    def __init__(self, config: StreamConfig, jpeg_quality: int, max_fps: float):
        self.config = config
        self.jpeg_quality = jpeg_quality
        self.max_fps = max_fps
        self.lock = threading.Lock()
        self.condition = threading.Condition(self.lock)
        self.latest_jpeg: bytes | None = None
        self.latest_sequence = 0
        self.updated_at: float | None = None
        self.source = "waiting"
        self.running = True
        self.last_raw_encode_at = 0.0

    def _fps_allowed(self, now: float) -> bool:
        if self.max_fps <= 0.0 or not math.isfinite(self.max_fps):
            return True
        return now - self.last_raw_encode_at >= 1.0 / self.max_fps

    def update_compressed(self, payload: bytes):
        with self.condition:
            self.latest_jpeg = bytes(payload)
            self.latest_sequence += 1
            self.updated_at = time.time()
            self.source = "compressed"
            self.condition.notify_all()

    def update_raw(self, frame) -> bool:
        now = time.monotonic()
        with self.lock:
            compressed_recent = self.source == "compressed" and self.updated_at is not None and time.time() - self.updated_at < 2.0
            if compressed_recent or not self._fps_allowed(now):
                return False
            self.last_raw_encode_at = now
        ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        if not ok:
            return False
        with self.condition:
            self.latest_jpeg = encoded.tobytes()
            self.latest_sequence += 1
            self.updated_at = time.time()
            self.source = "raw fallback"
            self.condition.notify_all()
        return True

    def snapshot(self) -> bytes | None:
        with self.lock:
            return self.latest_jpeg

    def wait_for_frame_after(self, sequence: int, timeout: float = 2.0) -> tuple[bytes | None, int]:
        deadline = time.monotonic() + timeout
        with self.condition:
            while self.running and self.latest_sequence <= sequence:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self.condition.wait(timeout=remaining)
            return self.latest_jpeg, self.latest_sequence

    def status(self) -> dict[str, Any]:
        with self.lock:
            return {
                "id": self.config.stream_id,
                "label": self.config.label,
                "compressed_topic": self.config.compressed_topic,
                "raw_topic": self.config.raw_topic,
                "sequence": self.latest_sequence,
                "updated_at": self.updated_at,
                "status": self.source if self.latest_jpeg is not None else "waiting",
            }

    def stop(self):
        with self.condition:
            self.running = False
            self.condition.notify_all()


class LocalizationWebDebugNode(Node):
    def __init__(self):
        super().__init__("localization_web_debug_server")
        self.declare_parameter("web_host", "127.0.0.1")
        self.declare_parameter("web_port", 8080)
        self.declare_parameter("jpeg_quality", 80)
        self.declare_parameter("max_fps", 5.0)
        self.declare_parameter("stream_overrides_json", "{}")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("pose_topic", "/amcl_pose")
        self.declare_parameter("particle_topic", "/particlecloud")
        self.declare_parameter("debug_pointcloud_topic", "/field_line_observations_debug")
        self.declare_parameter("processing_time_topic", "/amcl_fusion/processing_time_ms")
        self.declare_parameter(
            "state_file",
            str(Path.home() / ".ros" / "rcj_localization_web_debug_state.json"),
        )

        self.web_host = str(self.get_parameter("web_host").value)
        self.web_port = int(self.get_parameter("web_port").value)
        self.jpeg_quality = max(1, min(100, int(self.get_parameter("jpeg_quality").value)))
        self.max_fps = float(self.get_parameter("max_fps").value)
        self.map_topic = str(self.get_parameter("map_topic").value)
        self.pose_topic = str(self.get_parameter("pose_topic").value)
        self.particle_topic = str(self.get_parameter("particle_topic").value)
        self.debug_pointcloud_topic = str(self.get_parameter("debug_pointcloud_topic").value)
        self.processing_time_topic = str(self.get_parameter("processing_time_topic").value)
        self.state_file = Path(str(self.get_parameter("state_file").value)).expanduser()
        self.bridge = CvBridge()
        stream_configs = self._load_stream_configs()
        self.streams = {
            config.stream_id: StreamState(config, self.jpeg_quality, self.max_fps)
            for config in stream_configs
        }
        self.active_stream_lock = threading.Lock()
        self.active_stream_id = "hsv_overlay"
        if self.active_stream_id not in self.streams:
            self.active_stream_id = next(iter(self.streams))
        self.active_stream_subscriptions = {}
        self.parameter_lock = threading.Lock()
        self.display_lock = threading.Lock()
        self.display_state = self._load_display_state()
        self.localization_lock = threading.Lock()
        self.localization = {
            "map": None,
            "pose": None,
            "particles": [],
            "particle_count": 0,
            "pointcloud_points": 0,
            "processing_time_ms": None,
        }

        self.set_active_stream(self.active_stream_id, auto_enable=False)
        self.stream_fallback_timer = self.create_timer(1.0, self._sync_active_stream_fallback)
        self._setup_localization_subscriptions()
        self.web_server = ThreadingHTTPServer((self.web_host, self.web_port), make_handler(self))
        self.web_server.daemon_threads = True
        self.web_thread = threading.Thread(
            target=self.web_server.serve_forever,
            name="localization_web_debug_http",
            daemon=True,
        )
        self.web_thread.start()
        self.get_logger().info(
            f"Localization web debug listening on http://{self.web_host}:{self.web_port}/"
        )

    def _load_stream_configs(self) -> list[StreamConfig]:
        configs = {config.stream_id: config for config in STREAMS}
        raw_overrides = str(self.get_parameter("stream_overrides_json").value or "{}")
        try:
            overrides = json.loads(raw_overrides)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f"Ignoring invalid stream_overrides_json: {exc}")
            return list(configs.values())
        if not isinstance(overrides, dict):
            self.get_logger().warn("Ignoring stream_overrides_json because it is not a JSON object.")
            return list(configs.values())
        for stream_id, override in overrides.items():
            if stream_id not in configs or not isinstance(override, dict):
                continue
            current = configs[stream_id]
            configs[stream_id] = StreamConfig(
                current.stream_id,
                str(override.get("label", current.label)),
                str(override.get("compressed_topic", current.compressed_topic)),
                str(override.get("raw_topic", current.raw_topic)),
            )
        return list(configs.values())

    def _load_display_state(self) -> dict[str, bool]:
        default_state = {"localization_visible": True}
        try:
            with self.state_file.open("r", encoding="utf-8") as state_file:
                loaded = json.load(state_file)
        except FileNotFoundError:
            return default_state
        except Exception as exc:
            self.get_logger().warn(f"Ignoring unreadable display state file: {exc}")
            return default_state
        if not isinstance(loaded, dict):
            return default_state
        return {
            "localization_visible": bool(
                loaded.get("localization_visible", default_state["localization_visible"])
            )
        }

    def _save_display_state(self) -> None:
        self.state_file.parent.mkdir(parents=True, exist_ok=True)
        tmp_path = self.state_file.with_suffix(self.state_file.suffix + ".tmp")
        with tmp_path.open("w", encoding="utf-8") as state_file:
            json.dump(self.display_state, state_file, indent=2, sort_keys=True)
            state_file.write("\n")
        tmp_path.replace(self.state_file)

    def set_display_state(self, values: dict[str, Any]) -> dict[str, Any]:
        if "localization_visible" not in values:
            raise ValueError("'localization_visible' is required")
        with self.display_lock:
            self.display_state["localization_visible"] = bool(values["localization_visible"])
            self._save_display_state()
            display = dict(self.display_state)
        return {"ok": True, "display": display}

    def get_display_state(self) -> dict[str, bool]:
        with self.display_lock:
            return dict(self.display_state)

    def set_active_stream(self, stream_id: str, *, auto_enable: bool = True) -> dict[str, Any]:
        if stream_id not in self.streams:
            raise ValueError(f"Unknown stream '{stream_id}'")

        with self.active_stream_lock:
            if stream_id != self.active_stream_id:
                self._destroy_active_stream_subscriptions_locked()
                self.active_stream_id = stream_id
            self._ensure_active_compressed_subscription_locked()

        warnings = []
        if auto_enable:
            enable_request = STREAM_ENABLE_PARAMS.get(stream_id)
            if enable_request is not None:
                node_key, parameters = enable_request
                try:
                    self.set_remote_parameters(node_key, parameters)
                except Exception as exc:
                    warnings.append(str(exc))
        return {
            "ok": True,
            "active_stream": self.active_stream_id,
            "warnings": warnings,
        }

    def _destroy_active_stream_subscriptions_locked(self):
        for subscription in self.active_stream_subscriptions.values():
            self.destroy_subscription(subscription)
        self.active_stream_subscriptions = {}

    def _ensure_active_compressed_subscription_locked(self):
        if "compressed" in self.active_stream_subscriptions:
            return
        stream = self.streams[self.active_stream_id]
        self.active_stream_subscriptions["compressed"] = self.create_subscription(
            CompressedImage,
            stream.config.compressed_topic,
            lambda msg, s=stream: s.update_compressed(bytes(msg.data)),
            qos_profile_sensor_data,
        )

    def _ensure_active_raw_subscription_locked(self):
        if "raw" in self.active_stream_subscriptions:
            return
        stream = self.streams[self.active_stream_id]
        self.active_stream_subscriptions["raw"] = self.create_subscription(
            Image,
            stream.config.raw_topic,
            lambda msg, s=stream: self._raw_image_callback(msg, s),
            qos_profile_sensor_data,
        )

    def _remove_active_raw_subscription_locked(self):
        subscription = self.active_stream_subscriptions.pop("raw", None)
        if subscription is not None:
            self.destroy_subscription(subscription)

    def _sync_active_stream_fallback(self):
        with self.active_stream_lock:
            stream = self.streams[self.active_stream_id]
            now = time.time()
            compressed_recent = (
                stream.source == "compressed" and
                stream.updated_at is not None and
                now - stream.updated_at < 2.0
            )
            has_frame = stream.updated_at is not None
            if compressed_recent:
                self._remove_active_raw_subscription_locked()
            elif not has_frame or now - stream.updated_at > 2.0:
                self._ensure_active_raw_subscription_locked()

    def _setup_localization_subscriptions(self):
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.map_sub = self.create_subscription(OccupancyGrid, self.map_topic, self._map_callback, map_qos)
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, self.pose_topic, self._pose_callback, 10)
        self.particle_sub = self.create_subscription(PoseArray, self.particle_topic, self._particle_callback, 10)
        self.pointcloud_sub = self.create_subscription(PointCloud2, self.debug_pointcloud_topic, self._pointcloud_callback, qos_profile_sensor_data)
        self.processing_sub = self.create_subscription(Float32, self.processing_time_topic, self._processing_callback, 10)

    def _raw_image_callback(self, msg: Image, stream: StreamState):
        try:
            if msg.encoding in ("mono8", "8UC1"):
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
            else:
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            stream.update_raw(frame)
        except Exception as exc:
            self.get_logger().warn(f"Failed to convert {stream.config.raw_topic}: {exc}")

    def _map_callback(self, msg: OccupancyGrid):
        max_preview_side = 180
        width = int(msg.info.width)
        height = int(msg.info.height)
        step = max(1, math.ceil(max(width, height) / max_preview_side))
        preview = []
        data = list(msg.data)
        for y in range(0, height, step):
            row = y * width
            for x in range(0, width, step):
                preview.append(int(data[row + x]))
        with self.localization_lock:
            self.localization["map"] = {
                "width": math.ceil(width / step),
                "height": math.ceil(height / step),
                "resolution": float(msg.info.resolution) * step,
                "origin_x": float(msg.info.origin.position.x),
                "origin_y": float(msg.info.origin.position.y),
                "preview": preview,
                "updated_at": time.time(),
            }

    def _pose_callback(self, msg: PoseWithCovarianceStamped):
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        with self.localization_lock:
            self.localization["pose"] = {
                "x": float(msg.pose.pose.position.x),
                "y": float(msg.pose.pose.position.y),
                "yaw": yaw,
                "yaw_deg": math.degrees(yaw),
                "updated_at": time.time(),
            }

    def _particle_callback(self, msg: PoseArray):
        poses = msg.poses
        stride = max(1, math.ceil(len(poses) / 250))
        particles = [
            {"x": float(p.position.x), "y": float(p.position.y)}
            for p in poses[::stride]
        ]
        with self.localization_lock:
            self.localization["particles"] = particles
            self.localization["particle_count"] = len(poses)
            self.localization["particles_updated_at"] = time.time()

    def _pointcloud_callback(self, msg: PointCloud2):
        point_step = max(1, int(msg.point_step))
        points = len(msg.data) // point_step
        with self.localization_lock:
            self.localization["pointcloud_points"] = points
            self.localization["pointcloud_updated_at"] = time.time()

    def _processing_callback(self, msg: Float32):
        with self.localization_lock:
            self.localization["processing_time_ms"] = float(msg.data)
            self.localization["processing_updated_at"] = time.time()

    def status_payload(self) -> dict[str, Any]:
        display = self.get_display_state()
        if display["localization_visible"]:
            with self.localization_lock:
                localization = json.loads(json.dumps(self.localization))
        else:
            localization = {"enabled": False}
        with self.active_stream_lock:
            active_stream = self.active_stream_id
            active_subscription_kinds = set(self.active_stream_subscriptions.keys())
        stream_status = {}
        for stream_id, stream in self.streams.items():
            status = stream.status()
            status["subscribed"] = stream_id == active_stream
            status["subscription_kinds"] = (
                sorted(active_subscription_kinds) if stream_id == active_stream else []
            )
            stream_status[stream_id] = status
        return {
            "ok": True,
            "active_stream": active_stream,
            "display": display,
            "streams": stream_status,
            "localization": localization,
        }

    def get_parameters_payload(self) -> dict[str, Any]:
        values: dict[str, Any] = {}
        for key, names in PARAMETER_NAMES.items():
            values[key] = self.get_remote_parameters(key, names)
        return {"ok": True, "parameters": values}

    def get_remote_parameters(self, key: str, names: list[str]) -> dict[str, Any]:
        with self.parameter_lock:
            node_name = PARAM_NODES[key]
            client = self.create_client(GetParameters, f"{node_name}/get_parameters")
            if not client.wait_for_service(timeout_sec=0.15):
                return {}
            request = GetParameters.Request()
            request.names = names
            future = client.call_async(request)
            if not self._wait_for_future(future):
                return {}
            response = future.result()
        return {
            name: parameter_value_to_python(value)
            for name, value in zip(names, response.values)
            if value.type != ParameterType.PARAMETER_NOT_SET
        }

    def set_remote_parameters(self, key: str, values: dict[str, Any]) -> dict[str, Any]:
        if key not in PARAM_NODES:
            raise ValueError(f"Unknown parameter node '{key}'")
        with self.parameter_lock:
            node_name = PARAM_NODES[key]
            client = self.create_client(SetParameters, f"{node_name}/set_parameters")
            if not client.wait_for_service(timeout_sec=0.5):
                raise RuntimeError(f"Parameter service is not available for {node_name}")
            request = SetParameters.Request()
            request.parameters = [
                Parameter(name, parameter_type_for_value(value), value).to_parameter_msg()
                for name, value in values.items()
            ]
            future = client.call_async(request)
            if not self._wait_for_future(future, timeout=2.0):
                raise TimeoutError(f"Timed out setting parameters on {node_name}")
            response = future.result()
        failures = [
            f"{name}: {result.reason or 'rejected'}"
            for name, result in zip(values.keys(), response.results)
            if not result.successful
        ]
        if failures:
            raise RuntimeError("; ".join(failures))
        return {"ok": True, "message": f"Updated {len(values)} parameter(s) on {node_name}."}

    def export_payload(self) -> dict[str, Any]:
        params = self.get_parameters_payload()["parameters"]
        parts = []
        for key, values in params.items():
            for name in PARAMETER_NAMES.get(key, []):
                if name in values and not isinstance(values[name], (list, dict)):
                    prefix = "" if key == "hsv" else f"{key}_"
                    parts.append(f"{prefix}{name}:={format_launch_value(values[name])}")
        return {"ok": True, "launch_arguments": " ".join(parts)}

    def _wait_for_future(self, future, timeout: float = 1.0) -> bool:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if future.done():
                return True
            time.sleep(0.01)
        return future.done()

    def shutdown_web(self):
        for stream in self.streams.values():
            stream.stop()
        with self.active_stream_lock:
            self._destroy_active_stream_subscriptions_locked()
        self.web_server.shutdown()
        self.web_server.server_close()


def parameter_type_for_value(value: Any) -> Parameter.Type:
    if isinstance(value, bool):
        return Parameter.Type.BOOL
    if isinstance(value, int):
        return Parameter.Type.INTEGER
    if isinstance(value, float):
        return Parameter.Type.DOUBLE
    return Parameter.Type.STRING


def parameter_value_to_python(value) -> Any:
    if value.type == ParameterType.PARAMETER_BOOL:
        return bool(value.bool_value)
    if value.type == ParameterType.PARAMETER_INTEGER:
        return int(value.integer_value)
    if value.type == ParameterType.PARAMETER_DOUBLE:
        return float(value.double_value)
    if value.type == ParameterType.PARAMETER_STRING:
        return str(value.string_value)
    if value.type == ParameterType.PARAMETER_BOOL_ARRAY:
        return list(value.bool_array_value)
    if value.type == ParameterType.PARAMETER_INTEGER_ARRAY:
        return list(value.integer_array_value)
    if value.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
        return list(value.double_array_value)
    if value.type == ParameterType.PARAMETER_STRING_ARRAY:
        return list(value.string_array_value)
    return None


def format_launch_value(value: Any) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def normalize_stream_id(path_part: str) -> str:
    for suffix in (".mjpg", ".jpg", ".jpeg"):
        if path_part.endswith(suffix):
            return path_part[: -len(suffix)]
    return path_part


def make_handler(node: LocalizationWebDebugNode):
    class LocalizationWebHandler(BaseHTTPRequestHandler):
        server_version = "LocalizationWebDebug/1.0"

        def log_message(self, fmt, *args):
            return

        def send_bytes(self, payload: bytes, content_type: str, status=HTTPStatus.OK):
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(payload)

        def send_json(self, payload: dict[str, Any], status=HTTPStatus.OK):
            self.send_bytes(json.dumps(payload).encode("utf-8"), "application/json; charset=utf-8", status)

        def do_GET(self):
            parsed = urlparse(self.path)
            if parsed.path in ("/", "/index.html"):
                self.send_bytes(INDEX_HTML.encode("utf-8"), "text/html; charset=utf-8")
                return
            if parsed.path == "/api/status":
                self.send_json(node.status_payload())
                return
            if parsed.path == "/api/parameters":
                self.send_json(node.get_parameters_payload())
                return
            if parsed.path == "/api/display":
                self.send_json({"ok": True, "display": node.get_display_state()})
                return
            if parsed.path == "/api/stream":
                self.send_json({"ok": True, "active_stream": node.active_stream_id})
                return
            if parsed.path == "/api/export":
                self.send_json(node.export_payload())
                return
            if parsed.path.startswith("/snapshot/"):
                self.handle_snapshot(parsed.path.rsplit("/", 1)[-1])
                return
            if parsed.path.startswith("/stream/"):
                self.handle_stream(parsed.path.rsplit("/", 1)[-1])
                return
            self.send_error(HTTPStatus.NOT_FOUND)

        def do_POST(self):
            parsed = urlparse(self.path)
            if parsed.path not in ("/api/parameters", "/api/display", "/api/stream"):
                self.send_error(HTTPStatus.NOT_FOUND)
                return
            try:
                size = int(self.headers.get("Content-Length", "0"))
                body = json.loads(self.rfile.read(size).decode("utf-8") or "{}")
                if parsed.path == "/api/stream":
                    if not isinstance(body, dict):
                        raise ValueError("Request body must be an object")
                    self.send_json(node.set_active_stream(str(body.get("stream_id", ""))))
                    return
                if parsed.path == "/api/display":
                    if not isinstance(body, dict):
                        raise ValueError("Request body must be an object")
                    self.send_json(node.set_display_state(body))
                    return
                key = str(body.get("node", ""))
                parameters = body.get("parameters", {})
                if not isinstance(parameters, dict):
                    raise ValueError("'parameters' must be an object")
                self.send_json(node.set_remote_parameters(key, parameters))
            except Exception as exc:
                self.send_json({"ok": False, "error": str(exc)}, HTTPStatus.BAD_REQUEST)

        def handle_snapshot(self, stream_id: str):
            stream_id = normalize_stream_id(stream_id)
            stream = node.streams.get(stream_id)
            if stream is None:
                self.send_error(HTTPStatus.NOT_FOUND)
                return
            payload = stream.snapshot()
            if payload is None:
                self.send_error(HTTPStatus.CONFLICT, "No frame available")
                return
            self.send_bytes(payload, mimetypes.types_map.get(".jpg", "image/jpeg"))

        def handle_stream(self, stream_id: str):
            stream_id = normalize_stream_id(stream_id)
            stream = node.streams.get(stream_id)
            if stream is None:
                self.send_error(HTTPStatus.NOT_FOUND)
                return
            try:
                node.set_active_stream(stream_id, auto_enable=False)
            except Exception as exc:
                node.get_logger().warn(f"Failed to activate stream '{stream_id}': {exc}")
            query = parse_qs(urlparse(self.path).query)
            if query.get("snapshot") == ["1"]:
                self.handle_snapshot(stream_id)
                return
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            sequence = -1
            while stream.running:
                payload, sequence = stream.wait_for_frame_after(sequence)
                if payload is None:
                    continue
                try:
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(payload)}\r\n\r\n".encode("ascii"))
                    self.wfile.write(payload)
                    self.wfile.write(b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    break

    return LocalizationWebHandler


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationWebDebugNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown_web()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
