#!/usr/bin/env python3

from flask import Flask, render_template_string, jsonify, request, send_from_directory, Response
from flask_cors import CORS
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from std_msgs.msg import Float32, String, Bool
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image as RosImage
from nav2_msgs.action import NavigateToPose
from cv_bridge import CvBridge
import threading
import json
import os
import yaml
import subprocess
import math
import time
import cv2
import numpy as np
from datetime import datetime
import socket
import signal
import netifaces

# ── Camera topics ─────────────────────────────────────────────────────────
# Normal (raw) feeds
CAMERA_TOPIC_B7_NORMAL   = '/bumpy7/image_raw'
CAMERA_TOPIC_B5_NORMAL   = '/bumpy5/image_raw'

# Obstacle-annotated feeds (published by dynamic_obstacle_detector.py)
CAMERA_TOPIC_B7_OBSTACLE = '/bumpy7/obstacle_feed'
CAMERA_TOPIC_B5_OBSTACLE = '/bumpy5/obstacle_feed'

# ── Other config ───────────────────────────────────────────────────────────
CMD_VEL_TOPIC   = "/bumpy7/cmd_vel"
MAP_DIRECTORY   = os.path.expanduser("~/bumpy_ws/src/bumpy_slam/maps/")
WAYPOINTS_DIRECTORY = os.path.expanduser("~/bumpy_ws/src/bumpy_slam/waypoints/")
ROBOT_NAME      = "bumpy7"

slam_process = None
slam_running = False


# ── Utility helpers ────────────────────────────────────────────────────────

def get_local_ip():
    try:
        for interface in netifaces.interfaces():
            addrs = netifaces.ifaddresses(interface)
            if netifaces.AF_INET in addrs:
                for addr in addrs[netifaces.AF_INET]:
                    ip = addr['addr']
                    if not ip.startswith('127.') and ip.startswith('192.168.'):
                        return ip
        return socket.gethostbyname(socket.gethostname())
    except:
        return '127.0.0.1'

def load_waypoints(map_name):
    waypoint_file = os.path.join(WAYPOINTS_DIRECTORY, f"{map_name}_waypoints.json")
    if os.path.exists(waypoint_file):
        with open(waypoint_file, 'r') as f:
            return json.load(f)
    return []

def save_waypoints(map_name, waypoints):
    os.makedirs(WAYPOINTS_DIRECTORY, exist_ok=True)
    with open(os.path.join(WAYPOINTS_DIRECTORY, f"{map_name}_waypoints.json"), 'w') as f:
        json.dump(waypoints, f, indent=2)
    return True

def load_map_yaml(map_name):
    yaml_path = os.path.join(MAP_DIRECTORY, f"{map_name}.yaml")
    try:
        if not os.path.exists(yaml_path):
            return None
        with open(yaml_path, 'r') as f:
            return yaml.safe_load(f)
    except Exception as e:
        print(f"⚠️  Error loading map YAML: {e}")
        return None

def get_available_maps():
    maps = []
    try:
        os.makedirs(MAP_DIRECTORY, exist_ok=True)
        for file in os.listdir(MAP_DIRECTORY):
            if file.endswith('.yaml'):
                map_name = file.replace('.yaml', '')
                yaml_path = os.path.join(MAP_DIRECTORY, file)
                created = os.path.getctime(yaml_path)
                maps.append({'name': map_name,
                             'created': datetime.fromtimestamp(created).strftime('%Y-%m-%d %H:%M:%S')})
    except Exception as e:
        print(f"Error getting maps: {e}")
    maps.sort(key=lambda x: x['created'], reverse=True)
    return maps

def get_map_image_path(map_name):
    map_yaml = load_map_yaml(map_name)
    if map_yaml and 'image' in map_yaml:
        image_name = map_yaml['image']
        if not image_name.startswith('/'):
            return os.path.join(MAP_DIRECTORY, image_name)
        return image_name
    for ext in ['.png', '.pgm', '.jpg']:
        path = os.path.join(MAP_DIRECTORY, f"{map_name}{ext}")
        if os.path.exists(path):
            return path
    return None

def start_slam_mapping():
    global slam_process, slam_running
    try:
        slam_process = subprocess.Popen(
            ['ros2', 'launch', 'slam_toolbox', 'online_async_launch.py', 'use_sim_time:=false'],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, preexec_fn=os.setsid)
        slam_running = True
        print("✅ SLAM mapping started")
        return True
    except Exception as e:
        print(f"❌ Error starting SLAM: {e}")
        return False

def stop_slam_mapping():
    global slam_process, slam_running
    try:
        if slam_process:
            os.killpg(os.getpgid(slam_process.pid), signal.SIGTERM)
            slam_process.wait(timeout=5)
            slam_process = None
        slam_running = False
        print("✅ SLAM mapping stopped")
        return True
    except Exception as e:
        print(f"❌ Error stopping SLAM: {e}")
        return False

def save_current_map(map_name):
    try:
        map_path = os.path.join(MAP_DIRECTORY, map_name)
        result = subprocess.run(
            ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', map_path,
             '--ros-args', '-p', 'save_map_timeout:=10000'],
            capture_output=True, text=True, timeout=15)
        if result.returncode == 0:
            print(f"✅ Map saved: {map_name}")
            return True
        print(f"❌ Map save failed: {result.stderr}")
        return False
    except Exception as e:
        print(f"❌ Error saving map: {e}")
        return False


# ── Flask app ──────────────────────────────────────────────────────────────
app = Flask(__name__)
CORS(app)

HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Bumpy7 - Robot Control System</title>
    <style>
        :root {
            --bg-gradient-1: #87CEEB; --bg-gradient-2: #4A90E2; --bg-gradient-3: #E0F6FF;
            --panel-bg: #ffffff; --panel-shadow: rgba(0,0,0,0.1);
            --text-primary: #333333; --text-secondary: #666666;
            --border-color: #e0e0e0; --control-bg: #f9f9f9; --control-hover: #f0f0f0;
            --canvas-bg: #f0f0f0; --canvas-border: #4A90E2;
            --header-gradient-1: #4A90E2; --header-gradient-2: #87CEEB;
            --slider-track: #e0e0e0; --slider-thumb: #4A90E2; --text-color: #212529;
        }
        [data-theme="dark"] {
            --bg-gradient-1: #1a1a2e; --bg-gradient-2: #16213e; --bg-gradient-3: #0f3460;
            --panel-bg: #1e1e1e; --panel-shadow: rgba(0,0,0,0.5);
            --text-primary: #e0e0e0; --text-secondary: #b0b0b0;
            --border-color: #3a3a3a; --control-bg: #2a2a2a; --control-hover: #333333;
            --canvas-bg: #1a1a1a; --canvas-border: #4A90E2;
            --header-gradient-1: #16213e; --header-gradient-2: #0f3460;
            --slider-track: #3a3a3a; --slider-thumb: #4dabf7; --text-color: #f8f9fa;
        }
        * { margin:0; padding:0; box-sizing:border-box; }
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg,var(--bg-gradient-1) 0%,var(--bg-gradient-2) 50%,var(--bg-gradient-3) 100%);
            min-height:100vh; display:flex; justify-content:center; align-items:center;
        }
        .login-container { background:var(--panel-bg); padding:40px; border-radius:20px; box-shadow:0 20px 60px var(--panel-shadow); width:400px; }
        .login-header { text-align:center; margin-bottom:30px; }
        .login-header h1 { color:#4A90E2; font-size:28px; margin-bottom:10px; }
        .login-header p { color:var(--text-secondary); font-size:14px; }
        .form-group { margin-bottom:20px; }
        .form-group label { display:block; margin-bottom:8px; color:var(--text-primary); font-weight:600; }
        .form-group input { width:100%; padding:12px; border:2px solid var(--border-color); border-radius:8px; font-size:14px; background:var(--panel-bg); color:var(--text-primary); }
        .form-group input:focus { outline:none; border-color:#4A90E2; }
        .login-btn { width:100%; padding:14px; background:linear-gradient(135deg,#4A90E2 0%,#87CEEB 100%); color:white; border:none; border-radius:8px; font-size:16px; font-weight:600; cursor:pointer; }
        .error-msg { background:#fee; color:#c33; padding:10px; border-radius:8px; margin-bottom:20px; display:none; }
        .dashboard { display:none; width:100vw; height:100vh; background:var(--panel-bg); }
        .header { background:linear-gradient(135deg,var(--header-gradient-1) 0%,var(--header-gradient-2) 100%); color:white; padding:15px 30px; display:flex; justify-content:space-between; align-items:center; box-shadow:0 2px 10px var(--panel-shadow); }
        .header h1 { font-size:24px; }
        .logout-btn { padding:8px 20px; background:rgba(255,255,255,0.2); color:white; border:2px solid white; border-radius:8px; cursor:pointer; font-weight:600; }
        .main-content { display:grid; grid-template-columns:350px 1fr 350px; gap:0; height:calc(100vh - 60px); overflow:hidden; background:var(--bg-gradient-3); }
        .left-panel,.right-panel { padding:20px; overflow-y:auto; background:var(--panel-bg); display:flex; flex-direction:column; gap:20px; }
        .left-panel { box-shadow:2px 0 10px var(--panel-shadow); }
        .right-panel { box-shadow:-2px 0 10px var(--panel-shadow); }
        .center-panel { display:flex; flex-direction:column; padding:20px; overflow:hidden; background:var(--panel-bg); }
        .control-section { background:var(--panel-bg); padding:20px; border-radius:12px; box-shadow:0 2px 8px var(--panel-shadow); border:1px solid var(--border-color); }
        .control-section h3 { color:#4A90E2; margin-bottom:15px; font-size:18px; border-bottom:2px solid #4A90E2; padding-bottom:8px; }
        .joystick-container { display:flex; flex-direction:column; align-items:center; gap:15px; }
        .teleop-grid { display:grid; grid-template-columns:repeat(3,1fr); gap:10px; width:100%; max-width:280px; }
        .teleop-btn { padding:20px; font-size:24px; background:linear-gradient(135deg,#4A90E2 0%,#87CEEB 100%); color:white; border:3px solid #2E5C8A; border-radius:12px; cursor:pointer; font-weight:600; transition:all 0.2s; user-select:none; box-shadow:0 4px 10px rgba(0,0,0,0.2); }
        .teleop-btn small { font-size:11px; opacity:.9; display:block; margin-top:4px; }
        .teleop-btn:hover { transform:scale(1.05); }
        .teleop-btn:active { transform:scale(0.95); }
        .teleop-btn.stop-btn { background:linear-gradient(135deg,#dc3545 0%,#c82333 100%); border-color:#bd2130; }
        .speed-control { display:flex; align-items:center; gap:10px; margin-top:10px; color:var(--text-color); }
        .speed-slider { flex:1; height:8px; border-radius:4px; background:var(--slider-track); outline:none; -webkit-appearance:none; }
        .speed-slider::-webkit-slider-thumb { -webkit-appearance:none; width:20px; height:20px; border-radius:50%; background:var(--slider-thumb); cursor:pointer; }
        .map-canvas-container { background:var(--panel-bg); border-radius:12px; padding:20px; box-shadow:0 4px 20px var(--panel-shadow); height:100%; display:flex; flex-direction:column; border:1px solid var(--border-color); }
        .top-controls { display:flex; justify-content:space-between; align-items:center; margin-bottom:15px; padding-bottom:15px; border-bottom:2px solid var(--border-color); flex-wrap:wrap; gap:10px; }
        .control-group { display:flex; gap:10px; align-items:center; flex-wrap:wrap; }
        #mapCanvas { flex:1; border:2px solid var(--canvas-border); border-radius:8px; cursor:crosshair; background:var(--canvas-bg); }
        .waypoint-actions-grid { display:grid; grid-template-columns:1fr 1fr; gap:8px; margin-bottom:15px; }
        .waypoint-list-header { padding:8px 0; color:var(--text-primary); font-size:14px; border-bottom:1px solid var(--border-color); margin-bottom:10px; }
        .btn { padding:10px 20px; border:none; border-radius:8px; cursor:pointer; font-weight:600; transition:all 0.2s; }
        .btn-primary { background:linear-gradient(135deg,#4A90E2 0%,#87CEEB 100%); color:white; }
        .btn-secondary { background:#6c757d; color:white; }
        .btn-success { background:#28a745; color:white; }
        .btn-danger { background:#dc3545; color:white; }
        .btn:hover { transform:translateY(-2px); box-shadow:0 4px 12px rgba(0,0,0,0.2); }
        .btn:disabled { opacity:.5; cursor:not-allowed; transform:none; }
        .status-indicator { display:inline-block; width:12px; height:12px; border-radius:50%; margin-right:8px; }
        .status-active { background:#28a745; animation:pulse 2s infinite; }
        .status-idle { background:#6c757d; }
        @keyframes pulse { 0%,100%{opacity:1} 50%{opacity:.5} }
        .slam-controls { display:flex; gap:10px; flex-wrap:wrap; }
        .info-badge { display:inline-block; padding:4px 12px; border-radius:12px; font-size:12px; font-weight:600; margin-top:5px; }
        .info-badge.success { background:#d4edda; color:#155724; }
        .info-badge.warning { background:#fff3cd; color:#856404; }
        .waypoint-list { max-height:200px; overflow-y:auto; }
        .waypoint-item { display:flex; justify-content:space-between; align-items:center; padding:8px; background:var(--control-bg); border-radius:6px; margin-bottom:8px; color:var(--text-primary); }
        .waypoint-item:hover { background:var(--control-hover); }
        .waypoint-actions { display:flex; gap:5px; }
        .icon-btn { padding:4px 8px; border:none; border-radius:4px; cursor:pointer; font-size:12px; }
        .debug-info { background:var(--control-bg); padding:10px; border-radius:8px; font-family:monospace; font-size:12px; margin-top:10px; color:var(--text-primary); border:1px solid var(--border-color); }
        .debug-info div { margin:3px 0; }
        .connection-status { display:flex; align-items:center; gap:8px; padding:8px 12px; background:rgba(255,255,255,0.2); border-radius:8px; font-size:14px; }
        .modal { display:none; position:fixed; z-index:1000; left:0; top:0; width:100%; height:100%; background:rgba(0,0,0,0.5); }
        .modal-content { background:var(--panel-bg); margin:15% auto; padding:30px; border-radius:12px; width:400px; box-shadow:0 4px 20px var(--panel-shadow); }
        .modal-header { font-size:20px; font-weight:600; color:#4A90E2; margin-bottom:20px; }
        .modal-input { width:100%; padding:12px; border:2px solid var(--border-color); border-radius:8px; font-size:14px; margin-bottom:20px; background:var(--panel-bg); color:var(--text-primary); }
        .modal-buttons { display:flex; gap:10px; justify-content:flex-end; }
        .theme-toggle { padding:8px 16px; background:rgba(255,255,255,0.2); color:white; border:2px solid white; border-radius:8px; cursor:pointer; font-weight:600; display:flex; align-items:center; gap:8px; }
        /* Camera */
        .camera-badge { display:inline-block; font-size:11px; padding:2px 8px; border-radius:10px; margin-left:8px; font-weight:600; vertical-align:middle; }
        .camera-badge.active { background:#d4edda; color:#155724; }
        .camera-badge.inactive { background:#f8d7da; color:#721c24; }
        .camera-badge.waiting { background:#fff3cd; color:#856404; }
        .camera-topic-info { font-size:11px; color:var(--text-secondary); margin-top:4px; font-family:monospace; }
        .camera-no-feed { padding:30px; text-align:center; color:var(--text-secondary); font-size:13px; background:var(--control-bg); border-radius:8px; border:2px dashed var(--border-color); }
        /* ✨ Feed mode toggle */
        .feed-mode-bar {
            display:flex; align-items:center; gap:0;
            background:var(--control-bg); border:2px solid var(--border-color);
            border-radius:10px; overflow:hidden; margin:8px 0;
        }
        .feed-mode-btn {
            flex:1; padding:8px 0; font-size:12px; font-weight:700;
            border:none; cursor:pointer; transition:all 0.2s;
            background:transparent; color:var(--text-secondary);
            letter-spacing:0.5px;
        }
        .feed-mode-btn.active-normal  { background:linear-gradient(135deg,#4A90E2,#87CEEB); color:#fff; }
        .feed-mode-btn.active-obstacle { background:linear-gradient(135deg,#dc3545,#e05c6a); color:#fff; }
        .feed-mode-btn:hover:not(.active-normal):not(.active-obstacle) { background:var(--control-hover); }
        /* robot cards */
        .robot-card { border:1px solid var(--border-color); border-radius:10px; overflow:hidden; }
        .robot-card-header { display:flex; justify-content:space-between; align-items:center; padding:10px 14px; background:var(--control-bg); border-bottom:1px solid var(--border-color); }
        .robot-card-name { font-weight:700; font-size:14px; color:var(--text-primary); }
        .robot-card-role { font-size:12px; font-weight:700; padding:3px 12px; border-radius:12px; }
        .robot-card-role.role-leader { background:linear-gradient(135deg,#FFD700,#FFA500); color:#5a3e00; }
        .robot-card-role.role-follower { background:#6c757d; color:#fff; }
        .robot-card-role.role-unknown { background:var(--border-color); color:var(--text-secondary); }
        .robot-card-body { padding:10px 14px; }
        .robot-card-row { display:flex; justify-content:space-between; font-size:13px; color:var(--text-primary); }
        .mini-bar-wrap { background:var(--slider-track); border-radius:4px; height:8px; margin-top:4px; overflow:hidden; }
        .mini-bar { height:100%; border-radius:4px; transition:width 0.5s, background 0.5s; }
        /* toggle switch */
        .toggle-switch { position:relative; display:inline-block; width:50px; height:24px; }
        .toggle-switch input { opacity:0; width:0; height:0; }
        .toggle-slider { position:absolute; cursor:pointer; top:0; left:0; right:0; bottom:0; background-color:#ccc; transition:0.4s; border-radius:24px; }
        .toggle-slider:before { position:absolute; content:""; height:18px; width:18px; left:3px; bottom:3px; background-color:white; transition:0.4s; border-radius:50%; }
        input:checked + .toggle-slider { background-color:#4A90E2; }
        input:checked + .toggle-slider:before { transform:translateX(26px); }
        /* collapsible */
        .section-header { display:flex; justify-content:space-between; align-items:center; cursor:pointer; user-select:none; padding:12px 16px; margin:-12px -16px 12px -16px; border-radius:8px; transition:background 0.2s; }
        .section-header:hover { background:var(--control-hover); }
        .section-header h3 { margin:0; display:flex; align-items:center; gap:8px; font-size:16px; }
        .expand-icon { font-size:16px; transition:transform 0.3s; display:inline-block; font-weight:bold; color:var(--text-secondary); }
        .expand-icon.collapsed { transform:rotate(-90deg); }
        .section-content { overflow:hidden; transition:max-height 0.4s cubic-bezier(0.4,0,0.2,1), opacity 0.3s ease-out, padding 0.3s ease-out; }
        .section-content.collapsed { max-height:0 !important; opacity:0; padding-top:0 !important; padding-bottom:0 !important; }
        .section-badge { font-size:10px; padding:3px 8px; border-radius:10px; background:linear-gradient(135deg,#4A90E2,#87CEEB); color:white; font-weight:600; margin-left:8px; }
        /* obstacle alert flash */
        .obstacle-alert { display:none; background:#dc3545; color:#fff; border-radius:8px; padding:6px 14px; font-size:12px; font-weight:700; animation:blink 1s step-start infinite; }
        @keyframes blink { 50%{opacity:0;} }
    </style>
</head>
<body>
    <div class="login-container" id="loginScreen">
        <div class="login-header">
            <h1>🤖 Bumpy7 Control</h1>
            <p>Advanced Robot Navigation System</p>
        </div>
        <div class="error-msg" id="loginError">Invalid credentials</div>
        <form id="loginForm">
            <div class="form-group"><label>Username</label><input type="text" id="username" required></div>
            <div class="form-group"><label>Password</label><input type="password" id="password" required></div>
            <button type="submit" class="login-btn">Login</button>
        </form>
    </div>

    <div class="dashboard" id="dashboard">
        <div class="header">
            <h1>🤖 Bumpy7 Control Dashboard</h1>
            <div style="display:flex;gap:15px;align-items:center;">
                <div class="connection-status" id="connectionStatus">
                    <span class="status-indicator status-idle"></span><span>Connecting...</span>
                </div>
                <button class="theme-toggle" onclick="toggleTheme()">
                    <span id="themeIcon">🌙</span><span id="themeText">Dark</span>
                </button>
                <button class="logout-btn" onclick="logout()">Logout</button>
            </div>
        </div>

        <div class="main-content">
            <!-- ═══════════════ LEFT PANEL ═══════════════ -->
            <div class="left-panel">
                <!-- Manual Control -->
                <div class="control-section">
                    <div class="section-header" onclick="toggleSection('teleop')">
                        <h3>🎮 Manual Control <span class="section-badge">WASD</span></h3>
                        <span class="expand-icon" id="icon-teleop">▼</span>
                    </div>
                    <div class="section-content" id="content-teleop" style="max-height:1000px;">
                    <div class="joystick-container">
                        <div class="teleop-grid">
                            <div></div>
                            <button class="teleop-btn" onmousedown="teleopStart('forward')" onmouseup="teleopStop()" ontouchstart="teleopStart('forward')" ontouchend="teleopStop()">▲<br><small>W</small></button>
                            <div></div>
                            <button class="teleop-btn" onmousedown="teleopStart('left')" onmouseup="teleopStop()" ontouchstart="teleopStart('left')" ontouchend="teleopStop()">◄<br><small>A</small></button>
                            <button class="teleop-btn stop-btn" onclick="emergencyStop()">⬛<br><small>STOP</small></button>
                            <button class="teleop-btn" onmousedown="teleopStart('right')" onmouseup="teleopStop()" ontouchstart="teleopStart('right')" ontouchend="teleopStop()">►<br><small>D</small></button>
                            <div></div>
                            <button class="teleop-btn" onmousedown="teleopStart('backward')" onmouseup="teleopStop()" ontouchstart="teleopStart('backward')" ontouchend="teleopStop()">▼<br><small>S</small></button>
                            <div></div>
                        </div>
                        <div class="speed-control">
                            <span>Speed:</span>
                            <input type="range" class="speed-slider" id="speedSlider" min="0.1" max="1.0" step="0.1" value="0.5">
                            <span id="speedValue">0.5</span>
                        </div>
                        <button class="btn btn-secondary" style="width:100%;margin-top:10px;" onclick="toggleJoystick()">Toggle Joystick</button>
                        <div class="joystick" id="joystick" style="display:none;margin-top:15px;width:200px;height:200px;background:linear-gradient(135deg,#f0f0f0,#e0e0e0);border-radius:50%;position:relative;border:3px solid #4A90E2;">
                            <div id="joystickHandle" style="width:80px;height:80px;background:linear-gradient(135deg,#4A90E2,#87CEEB);border-radius:50%;position:absolute;top:50%;left:50%;transform:translate(-50%,-50%);cursor:move;box-shadow:0 4px 15px rgba(74,144,226,0.4);"></div>
                        </div>
                    </div>
                    </div>
                </div>

                <!-- Robot Status -->
                <div class="control-section">
                    <div class="section-header" onclick="toggleSection('robotstatus')">
                        <h3>📊 Robot Status</h3>
                        <span class="expand-icon" id="icon-robotstatus">▼</span>
                    </div>
                    <div class="section-content" id="content-robotstatus" style="max-height:500px;">
                    <div class="debug-info">
                        <div><strong>Position:</strong></div>
                        <div>X: <span id="posX">0.00</span> m</div>
                        <div>Y: <span id="posY">0.00</span> m</div>
                        <div>θ: <span id="posTheta">0.00</span>°</div>
                        <div style="margin-top:8px;"><strong>Velocity:</strong></div>
                        <div>Linear: <span id="velLinear">0.00</span> m/s</div>
                        <div>Angular: <span id="velAngular">0.00</span> rad/s</div>
                    </div>
                    </div>
                </div>
            </div>

            <!-- ═══════════════ CENTER PANEL ═══════════════ -->
            <div class="center-panel">
                <div class="map-canvas-container">
                    <div class="top-controls">
                        <div class="control-group" style="flex:1;min-width:300px;">
                            <label style="margin-right:10px;color:var(--text-primary);font-weight:600;">🗺️ Map:</label>
                            <select id="mapSelectorDropdown" onchange="selectMapFromDropdown()" style="padding:8px 12px;border-radius:6px;border:1px solid var(--border-color);background:var(--control-bg);color:var(--text-primary);font-size:14px;min-width:200px;">
                                <option value="">Loading maps...</option>
                            </select>
                        </div>
                        <div class="control-group">
                            <button class="btn btn-primary" onclick="resetView()">🔄 Reset View</button>
                            <button class="btn btn-secondary" onclick="toggleLaserScan()" id="laserToggleBtn">👁️ Hide Laser</button>
                            <button class="btn btn-danger" id="navCancelBtn" onclick="cancelNavigation()" disabled>⛔ Cancel Nav</button>
                        </div>
                    </div>
                    <canvas id="mapCanvas"></canvas>
                </div>
            </div>

            <!-- ═══════════════ RIGHT PANEL ═══════════════ -->
            <div class="right-panel">
                <!-- SLAM Mapping -->
                <div class="control-section">
                    <div class="section-header" onclick="toggleSection('slam')">
                        <h3>🗺️ SLAM Mapping</h3>
                        <span class="expand-icon" id="icon-slam">▼</span>
                    </div>
                    <div class="section-content" id="content-slam" style="max-height:2000px;">
                    <div class="slam-controls">
                        <button class="btn btn-success" id="startSlamBtn" onclick="startSlam()">Start SLAM</button>
                        <button class="btn btn-danger" id="stopSlamBtn" onclick="stopSlam()" disabled>Stop SLAM</button>
                        <button class="btn btn-primary" id="saveMapBtn" onclick="showSaveMapModal()" disabled>Save Map</button>
                    </div>
                    <div id="slamStatus" class="info-badge success" style="display:none;margin-top:10px;">
                        <span class="status-indicator status-active"></span>SLAM Active
                    </div>

                    <!-- Robot cards -->
                    <div style="margin-top:16px;display:flex;flex-direction:column;gap:12px;">
                        <div class="robot-card">
                            <div class="robot-card-header">
                                <span class="robot-card-name">🤖 bumpy7</span>
                                <span class="robot-card-role role-unknown" id="role-bumpy7">⏳ --</span>
                            </div>
                            <div class="robot-card-body">
                                <div class="robot-card-row"><span>🔋 Battery</span><span id="batt-bumpy7">--%</span></div>
                                <div class="mini-bar-wrap"><div class="mini-bar" id="battbar-bumpy7" style="width:0%;background:#28a745;"></div></div>
                                <div class="robot-card-row" style="margin-top:8px;"><span>📡 Confidence</span><span id="conf-bumpy7">--</span></div>
                                <div class="mini-bar-wrap"><div class="mini-bar" id="confbar-bumpy7" style="width:0%;background:#4A90E2;"></div></div>
                            </div>
                        </div>
                        <div class="robot-card">
                            <div class="robot-card-header">
                                <span class="robot-card-name">🤖 bumpy5</span>
                                <span class="robot-card-role role-unknown" id="role-bumpy5">⏳ --</span>
                            </div>
                            <div class="robot-card-body">
                                <div class="robot-card-row"><span>🔋 Battery</span><span id="batt-bumpy5">--%</span></div>
                                <div class="mini-bar-wrap"><div class="mini-bar" id="battbar-bumpy5" style="width:0%;background:#28a745;"></div></div>
                                <div class="robot-card-row" style="margin-top:8px;"><span>📡 Confidence</span><span id="conf-bumpy5">--</span></div>
                                <div class="mini-bar-wrap"><div class="mini-bar" id="confbar-bumpy5" style="width:0%;background:#4A90E2;"></div></div>
                            </div>
                        </div>
                    </div>
                    </div>
                </div>

                <!-- Waypoint Navigation -->
                <div class="control-section">
                    <div class="section-header" onclick="toggleSection('waypoints')">
                        <h3>📍 Waypoint Navigation</h3>
                        <span class="expand-icon" id="icon-waypoints">▼</span>
                    </div>
                    <div class="section-content" id="content-waypoints" style="max-height:1000px;">
                    <div class="waypoint-actions-grid">
                        <button class="btn btn-primary" onclick="addWaypointMode()">➕ Add Waypoint</button>
                        <button class="btn btn-success" id="startNavBtn" onclick="startWaypointNavigation()" disabled>▶️ Start Nav</button>
                        <button class="btn btn-secondary" onclick="loadWaypointsFromFile()">📂 Load</button>
                        <button class="btn btn-secondary" onclick="saveWaypointsToFile()">💾 Save</button>
                        <button class="btn btn-danger" onclick="clearAllWaypoints()">🗑️ Clear All</button>
                    </div>
                    <div class="waypoint-list-header"><strong>Waypoints (<span id="waypointCount">0</span>)</strong></div>
                    <div class="waypoint-list" id="waypointList"><div class="info-badge warning">No waypoints added</div></div>
                    <div id="navStatusInfo" class="info-badge" style="display:none;margin-top:10px;">
                        <span class="status-indicator"></span><span id="navStatusText">Ready</span>
                    </div>
                    </div>
                </div>

                <!-- ✨ Camera Feed Section — bumpy7 -->
                <div class="control-section">
                    <div class="section-header" onclick="toggleSection('camera-b7')">
                        <h3>📹 bumpy7 Camera <span class="camera-badge waiting" id="cameraBadge-b7">Disabled</span></h3>
                        <span class="expand-icon" id="icon-camera-b7">▼</span>
                    </div>
                    <div class="section-content" id="content-camera-b7" style="max-height:900px;">
                        <div class="camera-topic-info" id="cameraTopicInfo-b7">Topic: —</div>

                        <div style="margin:8px 0;display:flex;align-items:center;justify-content:space-between;background:var(--control-bg);padding:10px;border-radius:8px;">
                            <span style="font-weight:600;">Enable Camera:</span>
                            <label class="toggle-switch">
                                <input type="checkbox" id="cameraToggle-b7" onchange="toggleCamera('b7', this.checked)">
                                <span class="toggle-slider"></span>
                            </label>
                        </div>

                        <!-- ✨ Feed mode buttons -->
                        <div class="feed-mode-bar" id="feedModeBar-b7">
                            <button class="feed-mode-btn active-normal" id="modeBtn-b7-normal"
                                    onclick="setFeedMode('b7','normal')">📷 Normal</button>
                            <button class="feed-mode-btn" id="modeBtn-b7-obstacle"
                                    onclick="setFeedMode('b7','obstacle')">🚨 Obstacle</button>
                        </div>
                        <span class="obstacle-alert" id="obstAlert-b7">⚠️ OBSTACLE DETECTED</span>

                        <div style="text-align:center;margin-top:8px;">
                            <img id="cameraFeedImg-b7" src="" style="width:100%;border-radius:8px;border:2px solid var(--border-color);display:none;" alt="bumpy7 Feed" onload="onCameraLoad('b7')" onerror="onCameraError('b7')">
                            <div class="camera-no-feed" id="cameraNoFeed-b7">📷 Camera Disabled</div>
                        </div>
                    </div>
                </div>

                <!-- ✨ Camera Feed Section — bumpy5 -->
                <div class="control-section">
                    <div class="section-header" onclick="toggleSection('camera-b5')">
                        <h3>📹 bumpy5 Camera <span class="camera-badge waiting" id="cameraBadge-b5">Disabled</span></h3>
                        <span class="expand-icon" id="icon-camera-b5">▼</span>
                    </div>
                    <div class="section-content" id="content-camera-b5" style="max-height:900px;">
                        <div class="camera-topic-info" id="cameraTopicInfo-b5">Topic: —</div>

                        <div style="margin:8px 0;display:flex;align-items:center;justify-content:space-between;background:var(--control-bg);padding:10px;border-radius:8px;">
                            <span style="font-weight:600;">Enable Camera:</span>
                            <label class="toggle-switch">
                                <input type="checkbox" id="cameraToggle-b5" onchange="toggleCamera('b5', this.checked)">
                                <span class="toggle-slider"></span>
                            </label>
                        </div>

                        <div class="feed-mode-bar" id="feedModeBar-b5">
                            <button class="feed-mode-btn active-normal" id="modeBtn-b5-normal"
                                    onclick="setFeedMode('b5','normal')">📷 Normal</button>
                            <button class="feed-mode-btn" id="modeBtn-b5-obstacle"
                                    onclick="setFeedMode('b5','obstacle')">🚨 Obstacle</button>
                        </div>
                        <span class="obstacle-alert" id="obstAlert-b5">⚠️ OBSTACLE DETECTED</span>

                        <div style="text-align:center;margin-top:8px;">
                            <img id="cameraFeedImg-b5" src="" style="width:100%;border-radius:8px;border:2px solid var(--border-color);display:none;" alt="bumpy5 Feed" onload="onCameraLoad('b5')" onerror="onCameraError('b5')">
                            <div class="camera-no-feed" id="cameraNoFeed-b5">📷 Camera Disabled</div>
                        </div>
                    </div>
                </div>
            </div>
        </div>
    </div>

    <!-- Save Map Modal -->
    <div id="saveMapModal" class="modal">
        <div class="modal-content">
            <div class="modal-header">Save Current Map</div>
            <input type="text" id="mapNameInput" class="modal-input" placeholder="Enter map name">
            <div class="modal-buttons">
                <button class="btn btn-secondary" onclick="hideSaveMapModal()">Cancel</button>
                <button class="btn btn-success" onclick="saveMap()">Save</button>
            </div>
        </div>
    </div>

    <script>
        const CORRECT_USERNAME = 'bumpy7';
        const CORRECT_PASSWORD = 'bumpykkr';
        const UPDATE_INTERVAL  = 50;

        let currentMap=null, mapImage=null, mapMetadata=null;
        let robotPose={x:0,y:0,theta:0}, laserScan=[], showLaser=true;
        let waypoints=[], addingWaypoint=false, isNavigating=false;
        let lastRobotPos={x:0,y:0,theta:0}, robotMoving=false;
        let currentTheme='light';

        const collapsedSections = JSON.parse(localStorage.getItem('collapsedSections') || '{}');

        // ── Feed mode state ─────────────────────────────────────────────
        // mode: 'normal' | 'obstacle'
        const feedMode = { b7: 'normal', b5: 'normal' };
        const cameraEnabled = { b7: false, b5: false };

        // ── Section collapse ─────────────────────────────────────────────
        function toggleSection(id) {
            const content=document.getElementById('content-'+id), icon=document.getElementById('icon-'+id);
            if(!content||!icon)return;
            const collapsed=content.classList.contains('collapsed');
            if(collapsed){ content.classList.remove('collapsed'); icon.classList.remove('collapsed'); icon.textContent='▼'; delete collapsedSections[id]; }
            else { content.classList.add('collapsed'); icon.classList.add('collapsed'); icon.textContent='▶'; collapsedSections[id]=true; }
            localStorage.setItem('collapsedSections',JSON.stringify(collapsedSections));
        }
        function restoreCollapsedStates() {
            for(const id in collapsedSections) if(collapsedSections[id]) {
                const c=document.getElementById('content-'+id), i=document.getElementById('icon-'+id);
                if(c&&i){ c.classList.add('collapsed'); i.classList.add('collapsed'); i.textContent='▶'; }
            }
        }

        // ── Camera helpers ───────────────────────────────────────────────
        function videoFeedUrl(robot) {
            return `/video_feed/${robot}?t=${Date.now()}`;
        }

        function onCameraLoad(robot) {
            document.getElementById(`cameraBadge-${robot}`).textContent='Live';
            document.getElementById(`cameraBadge-${robot}`).className='camera-badge active';
            document.getElementById(`cameraFeedImg-${robot}`).style.display='block';
            document.getElementById(`cameraNoFeed-${robot}`).style.display='none';
        }
        function onCameraError(robot) {
            document.getElementById(`cameraBadge-${robot}`).textContent='No Feed';
            document.getElementById(`cameraBadge-${robot}`).className='camera-badge inactive';
            document.getElementById(`cameraFeedImg-${robot}`).style.display='none';
            document.getElementById(`cameraNoFeed-${robot}`).style.display='block';
            if(cameraEnabled[robot]) {
                setTimeout(()=>{ document.getElementById(`cameraFeedImg-${robot}`).src=videoFeedUrl(robot); }, 3000);
            }
        }

        async function toggleCamera(robot, enabled) {
            cameraEnabled[robot]=enabled;
            try {
                const res=await fetch(`/api/camera/${robot}/toggle`,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({enabled})});
                const data=await res.json();
                if(data.success) {
                    const badge=document.getElementById(`cameraBadge-${robot}`);
                    const img=document.getElementById(`cameraFeedImg-${robot}`);
                    const noFeed=document.getElementById(`cameraNoFeed-${robot}`);
                    if(enabled) {
                        badge.textContent='Enabling...'; badge.className='camera-badge waiting';
                        img.src=videoFeedUrl(robot);
                        noFeed.innerHTML='⏳ Waiting for camera feed...';
                    } else {
                        badge.textContent='Disabled'; badge.className='camera-badge inactive';
                        img.style.display='none'; img.src='';
                        noFeed.style.display='block'; noFeed.innerHTML='📷 Camera Disabled';
                    }
                }
            } catch(e){ console.error('Camera toggle error:',e); }
        }

        // ✨ Switch between Normal and Obstacle feed modes
        function setFeedMode(robot, mode) {
            feedMode[robot]=mode;
            // Update button styles
            const btnNormal   = document.getElementById(`modeBtn-${robot}-normal`);
            const btnObstacle = document.getElementById(`modeBtn-${robot}-obstacle`);
            btnNormal.className   = 'feed-mode-btn' + (mode==='normal'   ? ' active-normal'   : '');
            btnObstacle.className = 'feed-mode-btn' + (mode==='obstacle' ? ' active-obstacle' : '');

            // Tell the server which topic to subscribe to for this robot
            fetch(`/api/camera/${robot}/mode`,{
                method:'POST',
                headers:{'Content-Type':'application/json'},
                body:JSON.stringify({mode})
            });

            // Reload the MJPEG stream if camera is enabled
            if(cameraEnabled[robot]) {
                const img=document.getElementById(`cameraFeedImg-${robot}`);
                img.src=videoFeedUrl(robot);
            }
        }

        async function loadCameraInfo(robot) {
            try {
                const res=await fetch(`/api/camera/${robot}/info`);
                const data=await res.json();
                document.getElementById(`cameraTopicInfo-${robot}`).textContent='Topic: '+data.topic;
                const toggle=document.getElementById(`cameraToggle-${robot}`);
                if(toggle&&toggle.checked!==data.enabled) toggle.checked=data.enabled;
                const badge=document.getElementById(`cameraBadge-${robot}`);
                if(data.enabled&&data.receiving){ badge.textContent='Live'; badge.className='camera-badge active'; }
                else if(data.enabled){ badge.textContent='Waiting...'; badge.className='camera-badge waiting'; }
                else { badge.textContent='Disabled'; badge.className='camera-badge inactive'; }
            } catch(e){}
        }
        setInterval(()=>{ loadCameraInfo('b7'); loadCameraInfo('b5'); }, 2000);

        // ── Obstacle alert flash ─────────────────────────────────────────
        function updateObstacleAlert(robot, detected) {
            const el=document.getElementById(`obstAlert-${robot}`);
            if(detected && feedMode[robot]==='obstacle') { el.style.display='inline-block'; }
            else { el.style.display='none'; }
        }

        // ── Theme ────────────────────────────────────────────────────────
        function toggleTheme() {
            currentTheme=currentTheme==='light'?'dark':'light';
            document.documentElement.setAttribute('data-theme',currentTheme);
            document.getElementById('themeIcon').textContent=currentTheme==='dark'?'☀️':'🌙';
            document.getElementById('themeText').textContent=currentTheme==='dark'?'Light':'Dark';
            localStorage.setItem('theme',currentTheme); draw();
        }
        function loadTheme() {
            const t=localStorage.getItem('theme');
            if(t){ currentTheme=t; document.documentElement.setAttribute('data-theme',t);
                document.getElementById('themeIcon').textContent=t==='dark'?'☀️':'🌙';
                document.getElementById('themeText').textContent=t==='dark'?'Light':'Dark'; }
        }

        // ── Login ────────────────────────────────────────────────────────
        document.getElementById('loginForm').addEventListener('submit',function(e){
            e.preventDefault();
            if(document.getElementById('username').value===CORRECT_USERNAME&&
               document.getElementById('password').value===CORRECT_PASSWORD){
                document.getElementById('loginScreen').style.display='none';
                document.getElementById('dashboard').style.display='block';
                initDashboard();
            } else { document.getElementById('loginError').style.display='block'; }
        });
        function logout(){ document.getElementById('loginScreen').style.display='flex'; document.getElementById('dashboard').style.display='none'; ['username','password'].forEach(id=>document.getElementById(id).value=''); }

        // ── Init ─────────────────────────────────────────────────────────
        const canvas=document.getElementById('mapCanvas'), ctx=canvas.getContext('2d');

        function initDashboard(){
            loadTheme(); resizeCanvas(); restoreCollapsedStates();
            window.addEventListener('resize',resizeCanvas);
            initJoystick(); loadMaps(); startDataUpdates(); checkSlamStatus();
            loadCameraInfo('b7'); loadCameraInfo('b5');
        }
        function resizeCanvas(){
            const c=canvas.parentElement, t=c.querySelector('.top-controls');
            canvas.width=c.clientWidth-40; canvas.height=c.clientHeight-t.clientHeight-40;
        }

        // ── Joystick ─────────────────────────────────────────────────────
        function initJoystick(){
            const joystick=document.getElementById('joystick'),handle=document.getElementById('joystickHandle');
            const speedSlider=document.getElementById('speedSlider');
            speedSlider.addEventListener('input',e=>document.getElementById('speedValue').textContent=e.target.value);
            let isDragging=false,jRect=joystick.getBoundingClientRect();
            handle.addEventListener('mousedown',startDrag); handle.addEventListener('touchstart',startDrag);
            function startDrag(e){isDragging=true;jRect=joystick.getBoundingClientRect();
                document.addEventListener('mousemove',drag);document.addEventListener('mouseup',stopDrag);
                document.addEventListener('touchmove',drag);document.addEventListener('touchend',stopDrag);}
            function drag(e){if(!isDragging)return;
                const cx=e.touches?e.touches[0].clientX:e.clientX,cy=e.touches?e.touches[0].clientY:e.clientY;
                const cx0=jRect.left+jRect.width/2,cy0=jRect.top+jRect.height/2;
                let dx=cx-cx0,dy=cy-cy0;
                const maxR=jRect.width/2-40,dist=Math.sqrt(dx*dx+dy*dy);
                if(dist>maxR){const a=Math.atan2(dy,dx);dx=maxR*Math.cos(a);dy=maxR*Math.sin(a);}
                handle.style.transform=`translate(calc(-50% + ${dx}px),calc(-50% + ${dy}px))`;
                const speed=parseFloat(speedSlider.value);
                const linear=(-dy/maxR)*speed,angular=(-dx/maxR)*speed*2;
                sendVelocity(linear,angular);
                document.getElementById('velLinear').textContent=linear.toFixed(2);
                document.getElementById('velAngular').textContent=angular.toFixed(2);}
            function stopDrag(){isDragging=false;handle.style.transform='translate(-50%,-50%)';
                sendVelocity(0,0);document.getElementById('velLinear').textContent='0.00';
                document.getElementById('velAngular').textContent='0.00';
                ['mousemove','mouseup','touchmove','touchend'].forEach((ev,i)=>
                    document.removeEventListener(ev,[drag,stopDrag,drag,stopDrag][i]));}
        }

        let teleopInterval=null;
        function teleopStart(dir){
            const speed=parseFloat(document.getElementById('speedSlider').value);
            let linear=0,angular=0;
            if(dir==='forward')linear=speed;else if(dir==='backward')linear=-speed;
            else if(dir==='left')angular=speed*1.5;else if(dir==='right')angular=-speed*1.5;
            sendVelocity(linear,angular);
            document.getElementById('velLinear').textContent=linear.toFixed(2);
            document.getElementById('velAngular').textContent=angular.toFixed(2);
            teleopInterval=setInterval(()=>sendVelocity(linear,angular),100);}
        function teleopStop(){if(teleopInterval){clearInterval(teleopInterval);teleopInterval=null;}
            sendVelocity(0,0);document.getElementById('velLinear').textContent='0.00';document.getElementById('velAngular').textContent='0.00';}
        function emergencyStop(){teleopStop();sendVelocity(0,0);}
        function toggleJoystick(){const j=document.getElementById('joystick');j.style.display=j.style.display==='none'?'block':'none';}

        document.addEventListener('keydown',e=>{
            if(document.activeElement.tagName==='INPUT')return;
            const k=e.key.toLowerCase();
            if(['w','arrowup'].includes(k)){e.preventDefault();if(!teleopInterval)teleopStart('forward');}
            else if(['s','arrowdown'].includes(k)){e.preventDefault();if(!teleopInterval)teleopStart('backward');}
            else if(['a','arrowleft'].includes(k)){e.preventDefault();if(!teleopInterval)teleopStart('left');}
            else if(['d','arrowright'].includes(k)){e.preventDefault();if(!teleopInterval)teleopStart('right');}
            else if(k===' '){e.preventDefault();emergencyStop();}});
        document.addEventListener('keyup',e=>{
            if(document.activeElement.tagName==='INPUT')return;
            if(['w','s','a','d','arrowup','arrowdown','arrowleft','arrowright'].includes(e.key.toLowerCase()))
                {e.preventDefault();teleopStop();}});

        function sendVelocity(linear,angular){
            fetch('/api/cmd_vel',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({linear,angular})});}

        // ── Maps ─────────────────────────────────────────────────────────
        async function loadMaps(){
            try{const r=await fetch('/api/maps'),data=await r.json();
                const dd=document.getElementById('mapSelectorDropdown');dd.innerHTML='';
                if(!data.maps.length){dd.innerHTML='<option value="">No maps available</option>';return;}
                data.maps.forEach((m,i)=>{const o=document.createElement('option');
                    o.value=m.name;o.textContent=`${m.name} (${m.created})`;if(!i)o.selected=true;dd.appendChild(o);});
                if(data.maps.length)loadMap(data.maps[0].name);}
            catch(e){console.error('Error loading maps:',e);}}
        function selectMapFromDropdown(){const v=document.getElementById('mapSelectorDropdown').value;if(v)loadMap(v);}
        async function loadMap(mapName){
            try{currentMap=mapName;
                const dd=document.getElementById('mapSelectorDropdown');if(dd)dd.value=mapName;
                const img=new Image();img.onload=()=>{mapImage=img;};img.src=`/api/map/${mapName}?t=${Date.now()}`;
                const wr=await fetch(`/api/waypoints/${mapName}`),wd=await wr.json();
                waypoints=wd.waypoints||[];updateWaypointList();}
            catch(e){console.error('Error loading map:',e);}}

        // ── Data updates ─────────────────────────────────────────────────
        function startDataUpdates(){
            setInterval(async()=>{
                try{const r=await fetch('/api/data'),data=await r.json();
                    const dx=data.robot_pose.x-lastRobotPos.x,dy=data.robot_pose.y-lastRobotPos.y;
                    robotMoving=(Math.abs(dx)>0.001||Math.abs(dy)>0.001||Math.abs(data.robot_pose.theta-lastRobotPos.theta)>0.01);
                    lastRobotPos={...data.robot_pose};robotPose=data.robot_pose;
                    laserScan=data.laser_scan||[];mapMetadata=data.map_metadata;
                    document.getElementById('posX').textContent=robotPose.x.toFixed(2);
                    document.getElementById('posY').textContent=robotPose.y.toFixed(2);
                    document.getElementById('posTheta').textContent=(robotPose.theta*180/Math.PI).toFixed(1);

                    // bumpy7 card
                    const batt=data.battery_percentage;
                    if(batt!=null){document.getElementById('batt-bumpy7').textContent=batt.toFixed(1)+'%';
                        const bar=document.getElementById('battbar-bumpy7');
                        bar.style.width=Math.min(100,Math.max(0,batt))+'%';
                        bar.style.background=batt>50?'#28a745':batt>20?'#ffc107':'#dc3545';}
                    const conf=data.slam_confidence;
                    if(conf!=null){document.getElementById('conf-bumpy7').textContent=conf.toFixed(3);
                        document.getElementById('confbar-bumpy7').style.width=Math.min(100,Math.max(0,conf*100))+'%';}

                    // bumpy5 card
                    const b5=data.b5_battery;
                    if(b5!=null){document.getElementById('batt-bumpy5').textContent=b5.toFixed(1)+'%';
                        const bar=document.getElementById('battbar-bumpy5');
                        bar.style.width=Math.min(100,Math.max(0,b5))+'%';
                        bar.style.background=b5>50?'#28a745':b5>20?'#ffc107':'#dc3545';}
                    const c5=data.b5_confidence;
                    if(c5!=null){document.getElementById('conf-bumpy5').textContent=c5.toFixed(3);
                        document.getElementById('confbar-bumpy5').style.width=Math.min(100,Math.max(0,c5*100))+'%';}

                    // Role badges
                    const leader=data.current_leader, hasLeader=leader&&leader!=='null';
                    function setRoleBadge(id,isLeader,received){
                        const el=document.getElementById(id);
                        if(!received){el.textContent='⏳ --';el.className='robot-card-role role-unknown';return;}
                        if(isLeader){el.textContent='👑 Leader';el.className='robot-card-role role-leader';}
                        else{el.textContent='🤝 Follower';el.className='robot-card-role role-follower';}}
                    setRoleBadge('role-bumpy7',leader==='bumpy7',hasLeader);
                    setRoleBadge('role-bumpy5',leader==='bumpy5',hasLeader);

                    // Obstacle alerts
                    updateObstacleAlert('b7', data.b7_obstacle_detected);
                    updateObstacleAlert('b5', data.b5_obstacle_detected);

                    updateConnectionStatus(true);draw();}
                catch(e){updateConnectionStatus(false);}},UPDATE_INTERVAL);
            setInterval(checkNavigationStatus,500);}

        function updateConnectionStatus(ok){
            const s=document.getElementById('connectionStatus');
            s.querySelector('.status-indicator').className='status-indicator '+(ok?'status-active':'status-idle');
            s.querySelector('span:last-child').textContent=ok?'Connected':'Disconnected';}

        // ── Canvas draw ──────────────────────────────────────────────────
        function draw(){
            const isDark=currentTheme==='dark';
            ctx.clearRect(0,0,canvas.width,canvas.height);
            if(!mapImage||!mapMetadata){
                ctx.fillStyle=isDark?'#1a1a1a':'#f0f0f0';ctx.fillRect(0,0,canvas.width,canvas.height);
                ctx.fillStyle=isDark?'#888':'#999';ctx.font='20px Arial';ctx.textAlign='center';
                ctx.fillText('No map loaded',canvas.width/2,canvas.height/2);return;}
            const scale=Math.min(canvas.width/mapImage.width,canvas.height/mapImage.height)*0.9;
            const ox=(canvas.width-mapImage.width*scale)/2,oy=(canvas.height-mapImage.height*scale)/2;
            ctx.save();ctx.translate(ox,oy);ctx.scale(scale,scale);ctx.drawImage(mapImage,0,0);ctx.restore();
            function w2c(x,y){return{x:((x-mapMetadata.origin[0])/mapMetadata.resolution)*scale+ox,
                y:(mapImage.height-(y-mapMetadata.origin[1])/mapMetadata.resolution)*scale+oy};}
            if(showLaser&&laserScan.length){ctx.fillStyle='rgba(255,0,0,0.3)';
                laserScan.forEach(p=>{const c=w2c(p.x,p.y);ctx.beginPath();ctx.arc(c.x,c.y,2,0,Math.PI*2);ctx.fill();});}
            waypoints.forEach((wp,i)=>{const c=w2c(wp.x,wp.y);
                ctx.shadowColor='rgba(255,165,0,0.8)';ctx.shadowBlur=10;
                ctx.fillStyle='#FFA500';ctx.strokeStyle='#000';ctx.lineWidth=3;
                ctx.beginPath();ctx.arc(c.x,c.y,10,0,Math.PI*2);ctx.fill();ctx.stroke();
                ctx.shadowBlur=0;ctx.fillStyle='#000';ctx.strokeStyle='#FFF';ctx.lineWidth=3;
                ctx.font='bold 14px Arial';ctx.textAlign='center';ctx.textBaseline='middle';
                ctx.strokeText(i+1,c.x,c.y);ctx.fillText(i+1,c.x,c.y);});
            const rc=w2c(robotPose.x,robotPose.y);
            ctx.save();ctx.translate(rc.x,rc.y);ctx.rotate(robotPose.theta);
            const rs=22;
            ctx.shadowColor=robotMoving?'rgba(74,144,226,0.9)':'rgba(135,206,235,0.7)';ctx.shadowBlur=20;
            ctx.strokeStyle='#000';ctx.lineWidth=4;ctx.beginPath();ctx.arc(0,0,rs,0,Math.PI*2);ctx.stroke();
            ctx.shadowBlur=0;ctx.fillStyle=robotMoving?'#4A90E2':'#87CEEB';ctx.strokeStyle='#000';ctx.lineWidth=3;
            ctx.beginPath();ctx.arc(0,0,rs,0,Math.PI*2);ctx.fill();ctx.stroke();
            ctx.strokeStyle='#000';ctx.lineWidth=2;ctx.fillStyle='#FFF';
            ctx.beginPath();ctx.moveTo(rs*.7,0);ctx.lineTo(-rs*.3,rs*.5);ctx.lineTo(-rs*.3,-rs*.5);
            ctx.closePath();ctx.fill();ctx.stroke();ctx.restore();}

        function toggleLaserScan(){showLaser=!showLaser;document.getElementById('laserToggleBtn').textContent=showLaser?'👁️ Hide Laser':'👁️‍🗨️ Show Laser';}
        function resetView(){resizeCanvas();}

        // ── Waypoints ────────────────────────────────────────────────────
        function updateWaypointList(){
            const list=document.getElementById('waypointList'),count=document.getElementById('waypointCount');
            count.textContent=waypoints.length;document.getElementById('startNavBtn').disabled=!waypoints.length;
            if(!waypoints.length){list.innerHTML='<div class="info-badge warning">No waypoints added</div>';return;}
            list.innerHTML='';
            waypoints.forEach((wp,i)=>{const d=document.createElement('div');d.className='waypoint-item';
                d.innerHTML=`<span><strong>WP${i+1}</strong> (${wp.x.toFixed(2)}, ${wp.y.toFixed(2)})</span>
                    <div class="waypoint-actions">
                        <button class="icon-btn btn-primary" onclick="navigateToWaypoint(${i})">🎯</button>
                        <button class="icon-btn btn-danger" onclick="deleteWaypoint(${i})">✕</button>
                    </div>`;list.appendChild(d);});}

        function addWaypointMode(){addingWaypoint=true;updateNavStatus('Click on map to add waypoint','warning');}
        canvas.addEventListener('click',e=>{
            if(!addingWaypoint||!mapImage||!mapMetadata)return;
            const rect=canvas.getBoundingClientRect();
            const scale=Math.min(canvas.width/mapImage.width,canvas.height/mapImage.height)*0.9;
            const ox=(canvas.width-mapImage.width*scale)/2,oy=(canvas.height-mapImage.height*scale)/2;
            const mx=(e.clientX-rect.left-ox)/scale,my=(e.clientY-rect.top-oy)/scale;
            const wx=mx*mapMetadata.resolution+mapMetadata.origin[0];
            const wy=(mapImage.height-my)*mapMetadata.resolution+mapMetadata.origin[1];
            waypoints.push({x:wx,y:wy,theta:0});saveWaypoints();updateWaypointList();
            updateNavStatus(`Waypoint ${waypoints.length} added`,'success');addingWaypoint=false;});

        async function saveWaypoints(){if(!currentMap)return;
            await fetch('/api/waypoints/save',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({map_name:currentMap,waypoints})});}
        function deleteWaypoint(i){waypoints.splice(i,1);saveWaypoints();updateWaypointList();}
        async function navigateToWaypoint(i){const wp=waypoints[i];updateNavStatus(`Navigating to WP${i+1}...`,'active');
            await fetch('/api/navigate',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({x:wp.x,y:wp.y,theta:wp.theta})});}
        let currentWaypointIndex=0;
        async function startWaypointNavigation(){if(!waypoints.length){alert('No waypoints!');return;}
            currentWaypointIndex=0;updateNavStatus(`Starting (${waypoints.length} waypoints)`,'active');navigateToNextWaypoint();}
        async function navigateToNextWaypoint(){
            if(currentWaypointIndex>=waypoints.length){updateNavStatus('Navigation completed! ✅','success');return;}
            const wp=waypoints[currentWaypointIndex];
            updateNavStatus(`Going to WP${currentWaypointIndex+1} of ${waypoints.length}`,'active');
            await fetch('/api/navigate',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({x:wp.x,y:wp.y,theta:wp.theta})});
            currentWaypointIndex++;}
        function clearAllWaypoints(){if(!waypoints.length)return;if(confirm(`Delete all ${waypoints.length} waypoints?`)){waypoints=[];saveWaypoints();updateWaypointList();}}
        function saveWaypointsToFile(){if(!waypoints.length){alert('No waypoints!');return;}saveWaypoints();updateNavStatus(`${waypoints.length} waypoints saved`,'success');}
        async function loadWaypointsFromFile(){if(!currentMap){alert('Select a map first!');return;}
            try{const r=await fetch(`/api/waypoints/${currentMap}`),d=await r.json();
                if(d.waypoints&&d.waypoints.length){waypoints=d.waypoints;updateWaypointList();updateNavStatus(`${waypoints.length} waypoints loaded`,'success');}
                else alert('No saved waypoints for this map');}catch(e){alert('Error: '+e.message);}}
        function updateNavStatus(msg,type='idle'){
            const div=document.getElementById('navStatusInfo'),span=document.getElementById('navStatusText'),ind=div.querySelector('.status-indicator');
            div.style.display=type==='idle'?'none':'flex';div.style.alignItems='center';div.style.gap='8px';
            span.textContent=msg;div.classList.remove('info-badge','success','warning');ind.classList.remove('status-active','status-idle');
            if(type==='active'){div.classList.add('info-badge');div.style.background='#d1ecf1';div.style.color='#0c5460';ind.classList.add('status-active');}
            else if(type==='success'){div.classList.add('info-badge','success');ind.style.background='#28a745';}
            else if(type==='warning'){div.classList.add('info-badge','warning');ind.style.background='#ffc107';}}
        async function checkNavigationStatus(){try{const r=await fetch('/api/navigate/status'),d=await r.json();
            isNavigating=d.status==='active';document.getElementById('navCancelBtn').disabled=!isNavigating;}catch(e){}}
        async function cancelNavigation(){await fetch('/api/navigate/cancel',{method:'POST'});}

        // ── SLAM ─────────────────────────────────────────────────────────
        async function checkSlamStatus(){const r=await fetch('/api/slam/status'),d=await r.json();updateSlamUI(d.running);setTimeout(checkSlamStatus,2000);}
        async function startSlam(){const r=await fetch('/api/slam/start',{method:'POST'}),d=await r.json();if(d.success)updateSlamUI(true);else alert('Failed: '+(d.error||'?'));}
        async function stopSlam(){const r=await fetch('/api/slam/stop',{method:'POST'}),d=await r.json();if(d.success)updateSlamUI(false);}
        function updateSlamUI(running){
            document.getElementById('startSlamBtn').disabled=running;
            document.getElementById('stopSlamBtn').disabled=!running;
            document.getElementById('saveMapBtn').disabled=!running;
            document.getElementById('slamStatus').style.display=running?'block':'none';}
        function showSaveMapModal(){document.getElementById('saveMapModal').style.display='block';}
        function hideSaveMapModal(){document.getElementById('saveMapModal').style.display='none';document.getElementById('mapNameInput').value='';}
        async function saveMap(){
            const name=document.getElementById('mapNameInput').value.trim();
            if(!name){alert('Enter a map name');return;}
            const r=await fetch('/api/map/save',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({name})});
            const d=await r.json();
            if(d.success){alert('Map saved!');hideSaveMapModal();loadMaps();}else alert('Failed: '+(d.error||'?'));}
        window.onclick=e=>{if(e.target===document.getElementById('saveMapModal'))hideSaveMapModal();};
    </script>
</body>
</html>
"""


# ── ROS2 Node ──────────────────────────────────────────────────────────────

class CameraState:
    """Holds per-robot camera subscription + latest JPEG + mode."""
    TOPICS = {
        'b7': {
            'normal':   CAMERA_TOPIC_B7_NORMAL,
            'obstacle': CAMERA_TOPIC_B7_OBSTACLE,
        },
        'b5': {
            'normal':   CAMERA_TOPIC_B5_NORMAL,
            'obstacle': CAMERA_TOPIC_B5_OBSTACLE,
        },
    }

    def __init__(self, robot_id: str):
        self.robot_id        = robot_id
        self.mode            = 'normal'       # 'normal' | 'obstacle'
        self.enabled         = False
        self.receiving       = False
        self.latest_jpeg     = None
        self._lock           = threading.Lock()
        self.current_topic   = self.TOPICS[robot_id]['normal']

    @property
    def active_topic(self):
        return self.TOPICS[self.robot_id][self.mode]


class WebNode(Node):
    def __init__(self):
        super().__init__('web_interface_node')

        self.bridge = CvBridge()

        # ── Camera states ─────────────────────────────────────────────
        self.cam_b7 = CameraState('b7')
        self.cam_b5 = CameraState('b5')

        # ── Camera subscriptions (dynamically re-created on mode change) ──
        qos_cam = rclpy.qos.QoSProfile(
            depth=1,
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE,
        )
        self._sub_b7 = self.create_subscription(
            RosImage, self.cam_b7.active_topic,
            lambda msg: self._camera_cb(msg, self.cam_b7), qos_cam)
        self._sub_b5 = self.create_subscription(
            RosImage, self.cam_b5.active_topic,
            lambda msg: self._camera_cb(msg, self.cam_b5), qos_cam)

        # ── Obstacle detection status (from obstacle detector node) ───
        self.b7_obstacle_detected = False
        self.b5_obstacle_detected = False
        self.create_subscription(
            Bool,
            #rclpy.qos.QoSProfile(depth=5).__class__,  # placeholder import handled below
            '/bumpy7/obstacle_detected',
            lambda msg: setattr(self, 'b7_obstacle_detected', msg.data), 10)
        self.create_subscription(
            Bool,
            #rclpy.qos.QoSProfile(depth=5).__class__,
            '/bumpy5/obstacle_detected',
            lambda msg: setattr(self, 'b5_obstacle_detected', msg.data), 10)

        # ── Publishers ────────────────────────────────────────────────
        self.cmd_vel_pub       = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.initial_pose_pub  = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        # ── Nav2 action ───────────────────────────────────────────────
        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('⏳ Waiting for Nav2 action server...')
        if self.nav_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().info('✅ Nav2 action server connected!')
        else:
            self.get_logger().warn('⚠️  Nav2 action server not available yet')

        # ── Other subscribers ─────────────────────────────────────────
        self.odom_sub = self.create_subscription(Odometry, '/bumpy7/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/bumpy7/scan', self.laser_callback, 10)
        self.leader_sub = self.create_subscription(String, '/leader_ele', self.leader_callback, 10)

        # ── State ─────────────────────────────────────────────────────
        self.robot_pose        = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.laser_scan        = []
        self.map_metadata      = None
        self.navigation_active = False
        self.current_goal_handle = None
        self.battery_percentage  = None
        self.slam_confidence     = None
        self.current_leader      = None
        self.robot_role          = 'unknown'
        self.b5_battery          = None
        self.b5_confidence       = None

        self.get_logger().info(
            f'🚀 Web Interface Node started\n'
            f'  b7 normal   : {CAMERA_TOPIC_B7_NORMAL}\n'
            f'  b7 obstacle : {CAMERA_TOPIC_B7_OBSTACLE}\n'
            f'  b5 normal   : {CAMERA_TOPIC_B5_NORMAL}\n'
            f'  b5 obstacle : {CAMERA_TOPIC_B5_OBSTACLE}'
        )

    # ── Camera ────────────────────────────────────────────────────────
    def _camera_cb(self, msg: RosImage, cam: CameraState):
        if not cam.enabled:
            with cam._lock:
                cam.receiving = False
                cam.latest_jpeg = None
            return
        try:
            enc = msg.encoding.lower()
            if enc in ('rgb8', 'bgr8', 'mono8', '8uc1', '8uc3'):
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            else:
                try:
                    frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                except Exception:
                    frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            ret, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if ret:
                with cam._lock:
                    cam.latest_jpeg = buf.tobytes()
                    cam.receiving   = True
        except Exception as e:
            self.get_logger().warn(f'[{cam.robot_id}] Camera error: {e}', throttle_duration_sec=5.0)

    def set_camera_mode(self, robot_id: str, mode: str):
        """Switch between 'normal' and 'obstacle' topic for a robot's camera."""
        cam = self.cam_b7 if robot_id == 'b7' else self.cam_b5
        if mode not in ('normal', 'obstacle'):
            return False
        if cam.mode == mode:
            return True   # no-op

        cam.mode = mode
        new_topic = cam.active_topic

        # Re-subscribe to new topic
        qos_cam = rclpy.qos.QoSProfile(
            depth=1,
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE,
        )
        if robot_id == 'b7':
            self.destroy_subscription(self._sub_b7)
            self._sub_b7 = self.create_subscription(
                RosImage, new_topic,
                lambda msg: self._camera_cb(msg, self.cam_b7), qos_cam)
        else:
            self.destroy_subscription(self._sub_b5)
            self._sub_b5 = self.create_subscription(
                RosImage, new_topic,
                lambda msg: self._camera_cb(msg, self.cam_b5), qos_cam)

        # Clear stale frame
        with cam._lock:
            cam.latest_jpeg = None
            cam.receiving   = False
        self.get_logger().info(f'[{robot_id}] Camera switched to {mode} → {new_topic}')
        return True

    def get_latest_jpeg(self, robot_id: str):
        cam = self.cam_b7 if robot_id == 'b7' else self.cam_b5
        with cam._lock:
            if not cam.enabled:
                return None
            return cam.latest_jpeg

    # ── Nav2 ──────────────────────────────────────────────────────────
    def navigate_to_pose(self, x, y, theta=0):
        try:
            if not self.nav_action_client.server_is_ready():
                if not self.nav_action_client.wait_for_server(timeout_sec=2.0):
                    return False
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = float(x)
            goal_msg.pose.pose.position.y = float(y)
            goal_msg.pose.pose.position.z = 0.0
            qz = math.sin(theta / 2.0)
            qw = math.cos(theta / 2.0)
            goal_msg.pose.pose.orientation.z = qz
            goal_msg.pose.pose.orientation.w = qw
            send_goal_future = self.nav_action_client.send_goal_async(
                goal_msg, feedback_callback=self.navigation_feedback_callback)
            send_goal_future.add_done_callback(self.goal_response_callback)
            self.navigation_active = True
            return True
        except Exception as e:
            self.get_logger().error(f'Navigation failed: {e}')
            return False

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.navigation_active = False; return
            self.current_goal_handle = goal_handle
            goal_handle.get_result_async().add_done_callback(self.goal_result_callback)
        except Exception as e:
            self.get_logger().error(f'Goal response error: {e}')
            self.navigation_active = False

    def goal_result_callback(self, future):
        try: future.result()
        except Exception as e: self.get_logger().error(f'Navigation result error: {e}')
        finally: self.navigation_active = False; self.current_goal_handle = None

    def navigation_feedback_callback(self, feedback_msg): pass

    def cancel_navigation(self):
        if self.current_goal_handle is not None:
            try:
                self.current_goal_handle.cancel_goal_async().add_done_callback(
                    lambda f: self.get_logger().info('Navigation canceled'))
            except Exception as e:
                self.get_logger().error(f'Cancel error: {e}')
        self.publish_cmd_vel(0.0, 0.0)
        self.navigation_active = False
        return True

    # ── Odometry ──────────────────────────────────────────────────────
    def odom_callback(self, msg):
        self.robot_pose['x'] = msg.pose.pose.position.x
        self.robot_pose['y'] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_pose['theta'] = math.atan2(
            2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

    def laser_callback(self, msg):
        self.laser_scan = []
        step = max(1, len(msg.ranges) // 50)
        for i in range(0, len(msg.ranges), step):
            r = msg.ranges[i]
            if r < msg.range_min or r > msg.range_max or not math.isfinite(r): continue
            angle = msg.angle_min + i * msg.angle_increment + self.robot_pose['theta']
            self.laser_scan.append({
                'x': self.robot_pose['x'] + r * math.cos(angle),
                'y': self.robot_pose['y'] + r * math.sin(angle)})

    def leader_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.current_leader = data.get('leader', 'unknown')
            self.robot_role = 'leader' if self.current_leader == ROBOT_NAME else 'follower'
            bumpy7_data = data.get('bumpy7', {})
            if bumpy7_data:
                self.battery_percentage = bumpy7_data.get('battery', 0.0)
                self.slam_confidence    = bumpy7_data.get('covariance', 999.0)
            bumpy5_data = data.get('bumpy5', {})
            if bumpy5_data:
                self.b5_battery    = bumpy5_data.get('battery', 0.0)
                self.b5_confidence = bumpy5_data.get('covariance', 999.0)
        except Exception as e:
            self.get_logger().error(f'Leader callback error: {e}')

    def publish_cmd_vel(self, linear, angular):
        msg = Twist()
        msg.linear.x  = float(linear)
        msg.angular.z = float(angular)
        self.cmd_vel_pub.publish(msg)


# ── Global node reference ──────────────────────────────────────────────────
web_node: WebNode | None = None


# ── MJPEG stream (per-robot) ───────────────────────────────────────────────

def generate_frames(robot_id: str):
    while True:
        if web_node is None:
            time.sleep(0.05); continue
        jpeg = web_node.get_latest_jpeg(robot_id)
        if jpeg is None:
            cam = web_node.cam_b7 if robot_id == 'b7' else web_node.cam_b5
            placeholder = np.zeros((240, 320, 3), dtype=np.uint8)
            label = 'Camera Disabled' if not cam.enabled else f'Waiting for {cam.active_topic}...'
            cv2.putText(placeholder, label, (10, 110),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)
            cv2.putText(placeholder, f'Mode: {cam.mode}', (10, 140),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 200, 100), 1)
            _, buf = cv2.imencode('.jpg', placeholder)
            jpeg = buf.tobytes()
            time.sleep(0.5)
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpeg + b'\r\n')
        time.sleep(0.033)


# ── Flask routes ───────────────────────────────────────────────────────────

@app.route('/video_feed/<robot_id>')
def video_feed(robot_id):
    if robot_id not in ('b7', 'b5'):
        return 'Not found', 404
    return Response(generate_frames(robot_id),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

# ── Camera info ───────────────────────────────────────────────────────────
@app.route('/api/camera/<robot_id>/info')
def camera_info(robot_id):
    if not web_node or robot_id not in ('b7', 'b5'):
        return jsonify({'topic': '?', 'receiving': False, 'enabled': False, 'mode': 'normal'})
    cam = web_node.cam_b7 if robot_id == 'b7' else web_node.cam_b5
    return jsonify({'topic': cam.active_topic, 'receiving': cam.receiving,
                    'enabled': cam.enabled, 'mode': cam.mode})

# ── Camera toggle ─────────────────────────────────────────────────────────
@app.route('/api/camera/<robot_id>/toggle', methods=['POST'])
def toggle_camera(robot_id):
    if not web_node or robot_id not in ('b7', 'b5'):
        return jsonify({'success': False, 'error': 'Invalid robot or node not ready'}), 400
    data = request.get_json()
    enabled = data.get('enabled', False)
    cam = web_node.cam_b7 if robot_id == 'b7' else web_node.cam_b5
    cam.enabled = enabled
    if not enabled:
        with cam._lock:
            cam.latest_jpeg = None
            cam.receiving   = False
    return jsonify({'success': True, 'enabled': enabled})

# ✨ Camera mode switch (Normal <-> Obstacle)
@app.route('/api/camera/<robot_id>/mode', methods=['POST'])
def set_camera_mode(robot_id):
    if not web_node or robot_id not in ('b7', 'b5'):
        return jsonify({'success': False, 'error': 'Invalid robot or node not ready'}), 400
    data = request.get_json()
    mode = data.get('mode', 'normal')
    ok = web_node.set_camera_mode(robot_id, mode)
    cam = web_node.cam_b7 if robot_id == 'b7' else web_node.cam_b5
    return jsonify({'success': ok, 'mode': cam.mode, 'topic': cam.active_topic})

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

@app.route('/api/data')
def get_data():
    if web_node:
        return jsonify({
            'robot_pose':          web_node.robot_pose,
            'laser_scan':          web_node.laser_scan[:100],
            'map_metadata':        web_node.map_metadata,
            'battery_percentage':  web_node.battery_percentage,
            'slam_confidence':     web_node.slam_confidence,
            'robot_role':          web_node.robot_role,
            'current_leader':      web_node.current_leader,
            'b5_battery':          web_node.b5_battery,
            'b5_confidence':       web_node.b5_confidence,
            'b7_obstacle_detected': web_node.b7_obstacle_detected,
            'b5_obstacle_detected': web_node.b5_obstacle_detected,
        })
    return jsonify({'robot_pose': {'x':0,'y':0,'theta':0}, 'laser_scan': [], 'map_metadata': None})

@app.route('/api/cmd_vel', methods=['POST'])
def cmd_vel():
    try:
        data = request.get_json()
        if web_node:
            web_node.publish_cmd_vel(float(data.get('linear',0)), float(data.get('angular',0)))
            return jsonify({'success': True})
        return jsonify({'success': False, 'error': 'ROS node not initialized'}), 500
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 500

@app.route('/api/maps')
def get_maps():
    return jsonify({'maps': get_available_maps()})

@app.route('/api/map/<map_name>')
def get_map(map_name):
    try:
        map_path = get_map_image_path(map_name)
        if not map_path or not os.path.exists(map_path):
            return jsonify({'error': 'Map not found'}), 404
        if web_node: web_node.map_metadata = load_map_yaml(map_name)
        if map_path.endswith('.png'):
            return send_from_directory(os.path.dirname(map_path), os.path.basename(map_path), mimetype='image/png')
        elif map_path.endswith('.pgm'):
            png_path = map_path.replace('.pgm', '_temp.png')
            subprocess.run(['convert', map_path, png_path], check=True)
            return send_from_directory(os.path.dirname(png_path), os.path.basename(png_path), mimetype='image/png')
        return jsonify({'error': 'Unsupported format'}), 400
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/api/waypoints/<map_name>')
def get_waypoints_api(map_name):
    return jsonify({'waypoints': load_waypoints(map_name)})

@app.route('/api/waypoints/save', methods=['POST'])
def save_waypoints_api():
    data = request.get_json()
    return jsonify({'success': save_waypoints(data['map_name'], data['waypoints'])})

@app.route('/api/navigate', methods=['POST'])
def navigate():
    data = request.get_json()
    if web_node:
        return jsonify({'success': web_node.navigate_to_pose(data['x'], data['y'], data.get('theta',0))})
    return jsonify({'success': False})

@app.route('/api/navigate/cancel', methods=['POST'])
def cancel_nav():
    if web_node: return jsonify({'success': web_node.cancel_navigation()})
    return jsonify({'success': False})

@app.route('/api/navigate/status')
def nav_status():
    if web_node: return jsonify({'status': 'active' if web_node.navigation_active else 'idle'})
    return jsonify({'status': 'idle'})

@app.route('/api/slam/status')
def slam_status():
    return jsonify({'running': slam_running})

@app.route('/api/slam/start', methods=['POST'])
def start_slam():
    if slam_running: return jsonify({'success': False, 'error': 'Already running'})
    return jsonify({'success': start_slam_mapping()})

@app.route('/api/slam/stop', methods=['POST'])
def stop_slam():
    if not slam_running: return jsonify({'success': False, 'error': 'Not running'})
    return jsonify({'success': stop_slam_mapping()})

@app.route('/api/map/save', methods=['POST'])
def save_map():
    try:
        data = request.get_json()
        name = data.get('name','').strip()
        if not name: return jsonify({'success': False, 'error': 'Name required'})
        if not name.replace('_','').replace('-','').isalnum():
            return jsonify({'success': False, 'error': 'Invalid name'})
        ok = save_current_map(name)
        return jsonify({'success': ok, 'map_name': name} if ok else {'success': False, 'error': 'Save failed'})
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 500


# ── Entry point ────────────────────────────────────────────────────────────

def ros_spin():
    rclpy.spin(web_node)

def main():
    global web_node
    os.makedirs(MAP_DIRECTORY, exist_ok=True)
    os.makedirs(WAYPOINTS_DIRECTORY, exist_ok=True)

    rclpy.init()
    web_node = WebNode()

    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()

    print("\n" + "="*60)
    print("🤖 Bumpy7 Web Server")
    print("="*60)
    print(f"\n📹 bumpy7 normal   : {CAMERA_TOPIC_B7_NORMAL}")
    print(f"📹 bumpy7 obstacle : {CAMERA_TOPIC_B7_OBSTACLE}")
    print(f"📹 bumpy5 normal   : {CAMERA_TOPIC_B5_NORMAL}")
    print(f"📹 bumpy5 obstacle : {CAMERA_TOPIC_B5_OBSTACLE}")
    print(f"\n🌐 http://localhost:5000")
    print(f"🌐 http://{get_local_ip()}:5000")
    print("🔐 Login: bumpy7 / bumpykkr")
    print("="*60 + "\n")

    app.run(host='0.0.0.0', port=5000, debug=False)

if __name__ == '__main__':
    main()
