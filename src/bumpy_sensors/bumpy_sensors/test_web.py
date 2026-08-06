#!/usr/bin/env python3

from flask import Flask, render_template_string, jsonify, request, send_from_directory
from flask_cors import CORS
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import LaserScan
import threading
import json
import os
from pathlib import Path
import yaml
import subprocess
import math
import time
from datetime import datetime
import socket
import signal

import netifaces

def get_local_ip():
    """Dynamically detect local IP"""
    try:
        interfaces = netifaces.interfaces()
        for interface in interfaces:
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
    """Load waypoints for map"""
    waypoint_file = os.path.join(WAYPOINTS_DIRECTORY, f"{map_name}_waypoints.json")
    if os.path.exists(waypoint_file):
        with open(waypoint_file, 'r') as f:
            return json.load(f)
    return []

def save_waypoints(map_name, waypoints):
    """Save waypoints for map"""
    os.makedirs(WAYPOINTS_DIRECTORY, exist_ok=True)
    waypoint_file = os.path.join(WAYPOINTS_DIRECTORY, f"{map_name}_waypoints.json")
    with open(waypoint_file, 'w') as f:
        json.dump(waypoints, f, indent=2)
    return True
    
    
app = Flask(__name__)
CORS(app)


# Map file paths
MAP_DIRECTORY = os.path.expanduser("~/bumpy_ws/src/bumpy_slam/maps/")
WAYPOINTS_DIRECTORY = os.path.expanduser("~/bumpy_ws/src/bumpy_slam/waypoints/")
DEFAULT_MAP_NAME = "amr_map"

# CMD_VEL TOPIC CONFIGURATION - CHANGE THIS TO MATCH YOUR ROBOT
CMD_VEL_TOPIC = "/bumpy7/cmd_vel"

# SLAM process tracking
slam_process = None
slam_running = False

# Helper functions
def load_map_yaml(map_name):
    """Load map metadata from YAML file"""
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
    """Get list of available maps"""
    maps = []
    try:
        if not os.path.exists(MAP_DIRECTORY):
            os.makedirs(MAP_DIRECTORY, exist_ok=True)
        
        for file in os.listdir(MAP_DIRECTORY):
            if file.endswith('.yaml'):
                map_name = file.replace('.yaml', '')
                # Get file creation time
                yaml_path = os.path.join(MAP_DIRECTORY, file)
                created = os.path.getctime(yaml_path)
                maps.append({
                    'name': map_name,
                    'created': datetime.fromtimestamp(created).strftime('%Y-%m-%d %H:%M:%S')
                })
    except Exception as e:
        print(f"Error getting maps: {e}")
    
    # Sort by creation time (newest first)
    maps.sort(key=lambda x: x['created'], reverse=True)
    return maps

def get_map_image_path(map_name):
    """Get the path to the map image file"""
    map_yaml = load_map_yaml(map_name)
    if map_yaml and 'image' in map_yaml:
        image_name = map_yaml['image']
        # Handle both relative and full paths
        if not image_name.startswith('/'):
            return os.path.join(MAP_DIRECTORY, image_name)
        return image_name
    
    # Try common extensions
    for ext in ['.png', '.pgm', '.jpg']:
        path = os.path.join(MAP_DIRECTORY, f"{map_name}{ext}")
        if os.path.exists(path):
            return path
    
    return None

def start_slam_mapping():
    """Start SLAM mapping process"""
    global slam_process, slam_running
    
    try:
        # Launch slam_toolbox in mapping mode
        cmd = [
            'ros2', 'launch', 'slam_toolbox', 'online_async_launch.py',
            'use_sim_time:=false'
        ]
        
        slam_process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid  # Create new process group
        )
        
        slam_running = True
        print("✅ SLAM mapping started")
        return True
    except Exception as e:
        print(f"❌ Error starting SLAM: {e}")
        return False

def stop_slam_mapping():
    """Stop SLAM mapping process"""
    global slam_process, slam_running
    
    try:
        if slam_process:
            # Send SIGTERM to process group
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
    """Save the current map"""
    try:
        map_path = os.path.join(MAP_DIRECTORY, map_name)
        
        # Call map_saver service
        cmd = [
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
            '-f', map_path,
            '--ros-args', '-p', 'save_map_timeout:=10000'
        ]
        
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
        
        if result.returncode == 0:
            print(f"✅ Map saved: {map_name}")
            return True
        else:
            print(f"❌ Map save failed: {result.stderr}")
            return False
    except Exception as e:
        print(f"❌ Error saving map: {e}")
        return False

# HTML template with enhanced robot animations
HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Bumpy7 - Robot Control System</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #87CEEB 0%, #4A90E2 50%, #E0F6FF 100%);
            min-height: 100vh;
            display: flex;
            justify-content: center;
            align-items: center;
        }
        .login-container {
            background: white;
            padding: 40px;
            border-radius: 20px;
            box-shadow: 0 20px 60px rgba(0,0,0,0.3);
            width: 400px;
        }
        .login-header { text-align: center; margin-bottom: 30px; }
        .login-header h1 { color: #4A90E2; font-size: 28px; margin-bottom: 10px; }
        .login-header p { color: #666; font-size: 14px; }
        .form-group { margin-bottom: 20px; }
        .form-group label { display: block; margin-bottom: 8px; color: #333; font-weight: 600; }
        .form-group input {
            width: 100%;
            padding: 12px;
            border: 2px solid #e0e0e0;
            border-radius: 8px;
            font-size: 14px;
        }
        .form-group input:focus { outline: none; border-color: #4A90E2; }
        .login-btn {
            width: 100%;
            padding: 14px;
            background: linear-gradient(135deg, #4A90E2 0%, #87CEEB 100%);
            color: white;
            border: none;
            border-radius: 8px;
            font-size: 16px;
            font-weight: 600;
            cursor: pointer;
        }
        .error-msg {
            background: #fee;
            color: #c33;
            padding: 10px;
            border-radius: 8px;
            margin-bottom: 20px;
            display: none;
        }
        .dashboard {
            display: none;
            width: 100vw;
            height: 100vh;
            background: #f5f5f5;
        }
        .header {
            background: linear-gradient(135deg, #4A90E2 0%, #87CEEB 100%);
            color: white;
            padding: 15px 30px;
            display: flex;
            justify-content: space-between;
            align-items: center;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
        }
        .header h1 { font-size: 24px; }
        .logout-btn {
            padding: 8px 20px;
            background: rgba(255,255,255,0.2);
            color: white;
            border: 2px solid white;
            border-radius: 8px;
            cursor: pointer;
            font-weight: 600;
        }
        .main-content {
            display: flex;
            height: calc(100vh - 60px);
        }
        .left-panel {
            width: 350px;
            background: white;
            padding: 20px;
            overflow-y: auto;
            box-shadow: 2px 0 10px rgba(0,0,0,0.1);
        }
        .right-panel {
            flex: 1;
            padding: 20px;
            overflow: hidden;
        }
        .control-section {
            background: white;
            padding: 20px;
            border-radius: 12px;
            margin-bottom: 20px;
            box-shadow: 0 2px 8px rgba(0,0,0,0.1);
        }
        .control-section h3 {
            color: #4A90E2;
            margin-bottom: 15px;
            font-size: 18px;
            border-bottom: 2px solid #4A90E2;
            padding-bottom: 8px;
        }
        .joystick-container {
            display: flex;
            flex-direction: column;
            align-items: center;
            gap: 15px;
        }
        .joystick {
            width: 200px;
            height: 200px;
            background: linear-gradient(135deg, #f0f0f0 0%, #e0e0e0 100%);
            border-radius: 50%;
            position: relative;
            border: 3px solid #4A90E2;
            box-shadow: inset 0 2px 10px rgba(0,0,0,0.1);
        }
        .joystick-handle {
            width: 80px;
            height: 80px;
            background: linear-gradient(135deg, #4A90E2 0%, #87CEEB 100%);
            border-radius: 50%;
            position: absolute;
            top: 50%;
            left: 50%;
            transform: translate(-50%, -50%);
            cursor: move;
            box-shadow: 0 4px 15px rgba(74, 144, 226, 0.4);
            transition: box-shadow 0.2s;
        }
        .joystick-handle:hover {
            box-shadow: 0 6px 20px rgba(74, 144, 226, 0.6);
        }
        .speed-control {
            display: flex;
            align-items: center;
            gap: 10px;
            margin-top: 10px;
        }
        .speed-slider {
            flex: 1;
            height: 8px;
            border-radius: 4px;
            background: #e0e0e0;
            outline: none;
            -webkit-appearance: none;
        }
        .speed-slider::-webkit-slider-thumb {
            -webkit-appearance: none;
            appearance: none;
            width: 20px;
            height: 20px;
            border-radius: 50%;
            background: #4A90E2;
            cursor: pointer;
        }
        .map-selector {
            display: flex;
            flex-direction: column;
            gap: 10px;
        }
        .map-item {
            padding: 12px;
            background: #f9f9f9;
            border-radius: 8px;
            cursor: pointer;
            transition: all 0.2s;
            border: 2px solid transparent;
        }
        .map-item:hover {
            background: #f0f0f0;
            border-color: #4A90E2;
        }
        .map-item.active {
            background: linear-gradient(135deg, #4A90E2 0%, #87CEEB 100%);
            color: white;
            border-color: #4A90E2;
        }
        .map-canvas-container {
            background: white;
            border-radius: 12px;
            padding: 20px;
            box-shadow: 0 4px 20px rgba(0,0,0,0.1);
            height: 100%;
            display: flex;
            flex-direction: column;
        }
        #mapCanvas {
            flex: 1;
            border: 2px solid #4A90E2;
            border-radius: 8px;
            cursor: crosshair;
        }
        .canvas-controls {
            display: flex;
            gap: 10px;
            margin-top: 15px;
            flex-wrap: wrap;
        }
        .btn {
            padding: 10px 20px;
            border: none;
            border-radius: 8px;
            cursor: pointer;
            font-weight: 600;
            transition: all 0.2s;
        }
        .btn-primary {
            background: linear-gradient(135deg, #4A90E2 0%, #87CEEB 100%);
            color: white;
        }
        .btn-secondary {
            background: #6c757d;
            color: white;
        }
        .btn-success {
            background: #28a745;
            color: white;
        }
        .btn-danger {
            background: #dc3545;
            color: white;
        }
        .btn:hover {
            transform: translateY(-2px);
            box-shadow: 0 4px 12px rgba(0,0,0,0.2);
        }
        .btn:disabled {
            opacity: 0.5;
            cursor: not-allowed;
            transform: none;
        }
        .status-indicator {
            display: inline-block;
            width: 12px;
            height: 12px;
            border-radius: 50%;
            margin-right: 8px;
        }
        .status-active { background: #28a745; animation: pulse 2s infinite; }
        .status-idle { background: #6c757d; }
        @keyframes pulse {
            0%, 100% { opacity: 1; }
            50% { opacity: 0.5; }
        }
        
        /* ENHANCED ROBOT ANIMATIONS */
        .robot-marker {
            transition: all 0.3s cubic-bezier(0.4, 0.0, 0.2, 1);
            transform-origin: center;
            filter: drop-shadow(0 4px 8px rgba(74, 144, 226, 0.5));
        }
        
        .robot-marker.moving {
            animation: robotBounce 0.5s ease-in-out infinite alternate;
        }
        
        .robot-marker.rotating {
            animation: robotSpin 0.8s ease-in-out;
        }
        
        @keyframes robotBounce {
            0% { transform: scale(1) translateY(0); }
            100% { transform: scale(1.05) translateY(-2px); }
        }
        
        @keyframes robotSpin {
            0% { transform: rotate(0deg) scale(1); }
            50% { transform: rotate(180deg) scale(1.1); }
            100% { transform: rotate(360deg) scale(1); }
        }
        
        /* Robot pulse effect when idle */
        .robot-marker.idle {
            animation: robotPulse 2s ease-in-out infinite;
        }
        
        @keyframes robotPulse {
            0%, 100% { 
                filter: drop-shadow(0 4px 8px rgba(74, 144, 226, 0.5));
                transform: scale(1);
            }
            50% { 
                filter: drop-shadow(0 6px 12px rgba(74, 144, 226, 0.8));
                transform: scale(1.05);
            }
        }
        
        /* Trail effect for robot movement */
        .robot-trail {
            position: absolute;
            width: 4px;
            height: 4px;
            background: rgba(74, 144, 226, 0.3);
            border-radius: 50%;
            pointer-events: none;
            animation: trailFade 1s ease-out forwards;
        }
        
        @keyframes trailFade {
            0% {
                opacity: 1;
                transform: scale(1);
            }
            100% {
                opacity: 0;
                transform: scale(0.5);
            }
        }
        
        .slam-controls {
            display: flex;
            gap: 10px;
            flex-wrap: wrap;
        }
        .info-badge {
            display: inline-block;
            padding: 4px 12px;
            border-radius: 12px;
            font-size: 12px;
            font-weight: 600;
            margin-top: 5px;
        }
        .info-badge.success { background: #d4edda; color: #155724; }
        .info-badge.warning { background: #fff3cd; color: #856404; }
        .waypoint-list {
            max-height: 200px;
            overflow-y: auto;
        }
        .waypoint-item {
            display: flex;
            justify-content: space-between;
            align-items: center;
            padding: 8px;
            background: #f9f9f9;
            border-radius: 6px;
            margin-bottom: 8px;
        }
        .waypoint-item:hover {
            background: #f0f0f0;
        }
        .waypoint-actions {
            display: flex;
            gap: 5px;
        }
        .icon-btn {
            padding: 4px 8px;
            border: none;
            border-radius: 4px;
            cursor: pointer;
            font-size: 12px;
        }
        
        /* Position debug info */
        .debug-info {
            background: #f8f9fa;
            padding: 10px;
            border-radius: 8px;
            font-family: monospace;
            font-size: 12px;
            margin-top: 10px;
        }
        .debug-info div {
            margin: 3px 0;
        }
        
        /* Connection status indicator */
        .connection-status {
            display: flex;
            align-items: center;
            gap: 8px;
            padding: 8px 12px;
            background: #f8f9fa;
            border-radius: 8px;
            font-size: 14px;
        }
        
        .modal {
            display: none;
            position: fixed;
            z-index: 1000;
            left: 0;
            top: 0;
            width: 100%;
            height: 100%;
            background: rgba(0,0,0,0.5);
        }
        .modal-content {
            background: white;
            margin: 15% auto;
            padding: 30px;
            border-radius: 12px;
            width: 400px;
            box-shadow: 0 4px 20px rgba(0,0,0,0.3);
        }
        .modal-header {
            font-size: 20px;
            font-weight: 600;
            color: #4A90E2;
            margin-bottom: 20px;
        }
        .modal-input {
            width: 100%;
            padding: 12px;
            border: 2px solid #e0e0e0;
            border-radius: 8px;
            font-size: 14px;
            margin-bottom: 20px;
        }
        .modal-buttons {
            display: flex;
            gap: 10px;
            justify-content: flex-end;
        }
    </style>
</head>
<body>
    <!-- Login Screen -->
    <div class="login-container" id="loginScreen">
        <div class="login-header">
            <h1>🤖 Bumpy7 Control</h1>
            <p>Advanced Robot Navigation System</p>
        </div>
        <div class="error-msg" id="loginError">Invalid credentials</div>
        <form id="loginForm">
            <div class="form-group">
                <label>Username</label>
                <input type="text" id="username" required>
            </div>
            <div class="form-group">
                <label>Password</label>
                <input type="password" id="password" required>
            </div>
            <button type="submit" class="login-btn">Login</button>
        </form>
    </div>

    <!-- Dashboard -->
    <div class="dashboard" id="dashboard">
        <div class="header">
            <h1>🤖 Bumpy7 Control Dashboard</h1>
            <div style="display: flex; gap: 15px; align-items: center;">
                <div class="connection-status" id="connectionStatus">
                    <span class="status-indicator status-idle"></span>
                    <span>Connecting...</span>
                </div>
                <button class="logout-btn" onclick="logout()">Logout</button>
            </div>
        </div>

        <div class="main-content">
            <!-- Left Panel -->
            <div class="left-panel">
                <!-- Manual Control -->
                <div class="control-section">
                    <h3>🎮 Manual Control</h3>
                    <div class="joystick-container">
                        <div class="joystick" id="joystick">
                            <div class="joystick-handle" id="joystickHandle"></div>
                        </div>
                        <div class="speed-control">
                            <span>Speed:</span>
                            <input type="range" class="speed-slider" id="speedSlider" min="0.1" max="1.0" step="0.1" value="0.5">
                            <span id="speedValue">0.5</span>
                        </div>
                    </div>
                </div>

                <!-- Robot Status -->
                <div class="control-section">
                    <h3>📊 Robot Status</h3>
                    <div class="debug-info" id="robotStatus">
                        <div><strong>Position:</strong></div>
                        <div>X: <span id="posX">0.00</span> m</div>
                        <div>Y: <span id="posY">0.00</span> m</div>
                        <div>θ: <span id="posTheta">0.00</span>°</div>
                        <div style="margin-top: 8px;"><strong>Velocity:</strong></div>
                        <div>Linear: <span id="velLinear">0.00</span> m/s</div>
                        <div>Angular: <span id="velAngular">0.00</span> rad/s</div>
                    </div>
                </div>

                <!-- Map Selection -->
                <div class="control-section">
                    <h3>🗺️ Map Selection</h3>
                    <div class="map-selector" id="mapSelector">
                        <div class="info-badge warning">Loading maps...</div>
                    </div>
                </div>

                <!-- SLAM Controls -->
                <div class="control-section">
                    <h3>🗺️ SLAM Mapping</h3>
                    <div class="slam-controls">
                        <button class="btn btn-success" id="startSlamBtn" onclick="startSlam()">
                            Start SLAM
                        </button>
                        <button class="btn btn-danger" id="stopSlamBtn" onclick="stopSlam()" disabled>
                            Stop SLAM
                        </button>
                        <button class="btn btn-primary" id="saveMapBtn" onclick="showSaveMapModal()" disabled>
                            Save Map
                        </button>
                    </div>
                    <div id="slamStatus" class="info-badge success" style="display:none; margin-top:10px;">
                        <span class="status-indicator status-active"></span>
                        SLAM Active
                    </div>
                </div>

                <!-- Waypoint Navigation -->
                <div class="control-section">
                    <h3>📍 Waypoints</h3>
                    <div class="waypoint-list" id="waypointList">
                        <div class="info-badge warning">No waypoints</div>
                    </div>
                    <button class="btn btn-primary" style="width:100%; margin-top:10px;" onclick="addWaypointMode()">
                        Add Waypoint
                    </button>
                </div>
            </div>

            <!-- Right Panel - Map View -->
            <div class="right-panel">
                <div class="map-canvas-container">
                    <canvas id="mapCanvas"></canvas>
                    <div class="canvas-controls">
                        <button class="btn btn-primary" onclick="resetView()">Reset View</button>
                        <button class="btn btn-secondary" onclick="toggleLaserScan()">Toggle Laser</button>
                        <button class="btn btn-secondary" id="navCancelBtn" onclick="cancelNavigation()" disabled>
                            Cancel Navigation
                        </button>
                        <div style="flex:1"></div>
                        <div class="info-badge success" id="mapInfo">Map: None</div>
                    </div>
                </div>
            </div>
        </div>
    </div>

    <!-- Save Map Modal -->
    <div id="saveMapModal" class="modal">
        <div class="modal-content">
            <div class="modal-header">Save Current Map</div>
            <input type="text" id="mapNameInput" class="modal-input" placeholder="Enter map name (e.g., office_map)">
            <div class="modal-buttons">
                <button class="btn btn-secondary" onclick="hideSaveMapModal()">Cancel</button>
                <button class="btn btn-success" onclick="saveMap()">Save</button>
            </div>
        </div>
    </div>

    <script>
        // Configuration
        const CORRECT_USERNAME = 'bumpy7';
        const CORRECT_PASSWORD = 'bumpykkr';
        const UPDATE_INTERVAL = 50; // ms - smooth updates

        // State
        let currentMap = null;
        let mapImage = null;
        let mapMetadata = null;
        let robotPose = { x: 0, y: 0, theta: 0 };
        let laserScan = [];
        let showLaser = true;
        let waypoints = [];
        let addingWaypoint = false;
        let isNavigating = false;
        let lastRobotPos = { x: 0, y: 0, theta: 0 };
        let robotMoving = false;
        let robotTrails = [];

        // Canvas
        const canvas = document.getElementById('mapCanvas');
        const ctx = canvas.getContext('2d');
        
        // Joystick
        let joystickActive = false;
        let joystickPos = { x: 0, y: 0 };

        // Login
        document.getElementById('loginForm').addEventListener('submit', function(e) {
            e.preventDefault();
            const username = document.getElementById('username').value;
            const password = document.getElementById('password').value;
            
            if (username === CORRECT_USERNAME && password === CORRECT_PASSWORD) {
                document.getElementById('loginScreen').style.display = 'none';
                document.getElementById('dashboard').style.display = 'block';
                initDashboard();
            } else {
                document.getElementById('loginError').style.display = 'block';
            }
        });

        function logout() {
            document.getElementById('loginScreen').style.display = 'flex';
            document.getElementById('dashboard').style.display = 'none';
            document.getElementById('username').value = '';
            document.getElementById('password').value = '';
        }

        // Initialize Dashboard
        function initDashboard() {
            resizeCanvas();
            window.addEventListener('resize', resizeCanvas);
            initJoystick();
            loadMaps();
            startDataUpdates();
            checkSlamStatus();
        }

        function resizeCanvas() {
            const container = canvas.parentElement;
            const controls = container.querySelector('.canvas-controls');
            const availableHeight = container.clientHeight - controls.clientHeight - 40;
            canvas.width = container.clientWidth - 40;
            canvas.height = availableHeight;
        }

        // Joystick Control
        function initJoystick() {
            const joystick = document.getElementById('joystick');
            const handle = document.getElementById('joystickHandle');
            const speedSlider = document.getElementById('speedSlider');
            const speedValue = document.getElementById('speedValue');

            speedSlider.addEventListener('input', (e) => {
                speedValue.textContent = e.target.value;
            });

            let isDragging = false;
            let joystickRect = joystick.getBoundingClientRect();

            handle.addEventListener('mousedown', startDrag);
            handle.addEventListener('touchstart', startDrag);

            function startDrag(e) {
                isDragging = true;
                joystickRect = joystick.getBoundingClientRect();
                document.addEventListener('mousemove', drag);
                document.addEventListener('mouseup', stopDrag);
                document.addEventListener('touchmove', drag);
                document.addEventListener('touchend', stopDrag);
            }

            function drag(e) {
                if (!isDragging) return;

                const clientX = e.touches ? e.touches[0].clientX : e.clientX;
                const clientY = e.touches ? e.touches[0].clientY : e.clientY;

                const centerX = joystickRect.left + joystickRect.width / 2;
                const centerY = joystickRect.top + joystickRect.height / 2;

                let dx = clientX - centerX;
                let dy = clientY - centerY;

                const maxRadius = joystickRect.width / 2 - 40;
                const distance = Math.sqrt(dx * dx + dy * dy);

                if (distance > maxRadius) {
                    const angle = Math.atan2(dy, dx);
                    dx = maxRadius * Math.cos(angle);
                    dy = maxRadius * Math.sin(angle);
                }

                handle.style.transform = `translate(calc(-50% + ${dx}px), calc(-50% + ${dy}px))`;

                // Calculate velocity commands
                const speed = parseFloat(speedSlider.value);
                const normalizedX = dx / maxRadius;
                const normalizedY = -dy / maxRadius; // Invert Y

                const linear = normalizedY * speed;
                const angular = -normalizedX * speed * 2; // Turning sensitivity

                sendVelocity(linear, angular);
                
                // Update velocity display
                document.getElementById('velLinear').textContent = linear.toFixed(2);
                document.getElementById('velAngular').textContent = angular.toFixed(2);
            }

            function stopDrag() {
                isDragging = false;
                handle.style.transform = 'translate(-50%, -50%)';
                sendVelocity(0, 0);
                document.getElementById('velLinear').textContent = '0.00';
                document.getElementById('velAngular').textContent = '0.00';
                document.removeEventListener('mousemove', drag);
                document.removeEventListener('mouseup', stopDrag);
                document.removeEventListener('touchmove', drag);
                document.removeEventListener('touchend', stopDrag);
            }
        }

        function sendVelocity(linear, angular) {
            fetch('/api/cmd_vel', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ linear, angular })
            });
        }

        // Map Functions
        async function loadMaps() {
            try {
                const response = await fetch('/api/maps');
                const data = await response.json();
                
                const selector = document.getElementById('mapSelector');
                selector.innerHTML = '';

                if (data.maps.length === 0) {
                    selector.innerHTML = '<div class="info-badge warning">No maps available</div>';
                    return;
                }

                data.maps.forEach((map, index) => {
                    const div = document.createElement('div');
                    div.className = 'map-item' + (index === 0 ? ' active' : '');
                    div.innerHTML = `
                        <strong>${map.name}</strong><br>
                        <small>${map.created}</small>
                    `;
                    div.onclick = () => loadMap(map.name);
                    selector.appendChild(div);
                });

                // Load first map
                if (data.maps.length > 0) {
                    loadMap(data.maps[0].name);
                }
            } catch (error) {
                console.error('Error loading maps:', error);
            }
        }

        async function loadMap(mapName) {
            try {
                currentMap = mapName;
                
                // Update UI
                document.querySelectorAll('.map-item').forEach(item => {
                    item.classList.remove('active');
                    if (item.textContent.includes(mapName)) {
                        item.classList.add('active');
                    }
                });

                // Load map image
                const img = new Image();
                img.onload = () => {
                    mapImage = img;
                    document.getElementById('mapInfo').textContent = `Map: ${mapName}`;
                };
                img.src = `/api/map/${mapName}?t=${Date.now()}`;

                // Load waypoints
                const wpResponse = await fetch(`/api/waypoints/${mapName}`);
                const wpData = await wpResponse.json();
                waypoints = wpData.waypoints || [];
                updateWaypointList();

            } catch (error) {
                console.error('Error loading map:', error);
            }
        }

        // Data Updates
        function startDataUpdates() {
            setInterval(async () => {
                try {
                    const response = await fetch('/api/data');
                    const data = await response.json();

                    // Detect robot movement
                    const dx = data.robot_pose.x - lastRobotPos.x;
                    const dy = data.robot_pose.y - lastRobotPos.y;
                    const dtheta = Math.abs(data.robot_pose.theta - lastRobotPos.theta);
                    
                    robotMoving = (Math.abs(dx) > 0.001 || Math.abs(dy) > 0.001 || dtheta > 0.01);
                    
                    lastRobotPos = { ...data.robot_pose };
                    robotPose = data.robot_pose;
                    laserScan = data.laser_scan || [];
                    mapMetadata = data.map_metadata;

                    // Update status display
                    document.getElementById('posX').textContent = robotPose.x.toFixed(2);
                    document.getElementById('posY').textContent = robotPose.y.toFixed(2);
                    document.getElementById('posTheta').textContent = (robotPose.theta * 180 / Math.PI).toFixed(1);

                    updateConnectionStatus(true);
                    draw();
                } catch (error) {
                    updateConnectionStatus(false);
                    console.error('Error fetching data:', error);
                }
            }, UPDATE_INTERVAL);

            // Check navigation status
            setInterval(checkNavigationStatus, 500);
        }

        function updateConnectionStatus(connected) {
            const status = document.getElementById('connectionStatus');
            const indicator = status.querySelector('.status-indicator');
            const text = status.querySelector('span:last-child');
            
            if (connected) {
                indicator.className = 'status-indicator status-active';
                text.textContent = 'Connected';
            } else {
                indicator.className = 'status-indicator status-idle';
                text.textContent = 'Disconnected';
            }
        }

        // Drawing
        function draw() {
            ctx.clearRect(0, 0, canvas.width, canvas.height);

            if (!mapImage || !mapMetadata) {
                ctx.fillStyle = '#f0f0f0';
                ctx.fillRect(0, 0, canvas.width, canvas.height);
                ctx.fillStyle = '#999';
                ctx.font = '20px Arial';
                ctx.textAlign = 'center';
                ctx.fillText('No map loaded', canvas.width / 2, canvas.height / 2);
                return;
            }

            // Draw map
            const scale = Math.min(canvas.width / mapImage.width, canvas.height / mapImage.height) * 0.9;
            const offsetX = (canvas.width - mapImage.width * scale) / 2;
            const offsetY = (canvas.height - mapImage.height * scale) / 2;

            ctx.save();
            ctx.translate(offsetX, offsetY);
            ctx.scale(scale, scale);
            ctx.drawImage(mapImage, 0, 0);
            ctx.restore();

            // Transform coordinates
            function worldToCanvas(x, y) {
                const resolution = mapMetadata.resolution;
                const originX = mapMetadata.origin[0];
                const originY = mapMetadata.origin[1];

                const mapX = (x - originX) / resolution;
                const mapY = mapImage.height - (y - originY) / resolution;

                return {
                    x: mapX * scale + offsetX,
                    y: mapY * scale + offsetY
                };
            }

            // Draw laser scan
            if (showLaser && laserScan.length > 0) {
                ctx.fillStyle = 'rgba(255, 0, 0, 0.3)';
                laserScan.forEach(point => {
                    const canvasPoint = worldToCanvas(point.x, point.y);
                    ctx.beginPath();
                    ctx.arc(canvasPoint.x, canvasPoint.y, 2, 0, Math.PI * 2);
                    ctx.fill();
                });
            }

            // Draw waypoints
            waypoints.forEach((wp, index) => {
                const canvasPoint = worldToCanvas(wp.x, wp.y);
                
                // Waypoint circle
                ctx.fillStyle = '#FFA500';
                ctx.strokeStyle = '#FF8C00';
                ctx.lineWidth = 2;
                ctx.beginPath();
                ctx.arc(canvasPoint.x, canvasPoint.y, 8, 0, Math.PI * 2);
                ctx.fill();
                ctx.stroke();

                // Waypoint label
                ctx.fillStyle = 'white';
                ctx.font = 'bold 12px Arial';
                ctx.textAlign = 'center';
                ctx.textBaseline = 'middle';
                ctx.fillText((index + 1).toString(), canvasPoint.x, canvasPoint.y);
            });

            // Draw robot with enhanced animations
            const robotCanvas = worldToCanvas(robotPose.x, robotPose.y);
            
            ctx.save();
            ctx.translate(robotCanvas.x, robotCanvas.y);
            ctx.rotate(robotPose.theta);

            // Robot body (enhanced with animation classes effect)
            const robotSize = 20;
            
            // Glow effect when moving
            if (robotMoving) {
                ctx.shadowColor = 'rgba(74, 144, 226, 0.8)';
                ctx.shadowBlur = 15;
            }
            
            // Robot circle
            ctx.fillStyle = robotMoving ? '#4A90E2' : '#87CEEB';
            ctx.strokeStyle = '#2E5C8A';
            ctx.lineWidth = 3;
            ctx.beginPath();
            ctx.arc(0, 0, robotSize, 0, Math.PI * 2);
            ctx.fill();
            ctx.stroke();

            // Direction indicator
            ctx.fillStyle = '#FFF';
            ctx.beginPath();
            ctx.moveTo(robotSize * 0.6, 0);
            ctx.lineTo(-robotSize * 0.3, robotSize * 0.4);
            ctx.lineTo(-robotSize * 0.3, -robotSize * 0.4);
            ctx.closePath();
            ctx.fill();

            ctx.restore();
        }

        function toggleLaserScan() {
            showLaser = !showLaser;
        }

        function resetView() {
            resizeCanvas();
        }

        // Waypoint Functions
        function updateWaypointList() {
            const list = document.getElementById('waypointList');
            
            if (waypoints.length === 0) {
                list.innerHTML = '<div class="info-badge warning">No waypoints</div>';
                return;
            }

            list.innerHTML = '';
            waypoints.forEach((wp, index) => {
                const div = document.createElement('div');
                div.className = 'waypoint-item';
                div.innerHTML = `
                    <span><strong>WP${index + 1}</strong> (${wp.x.toFixed(2)}, ${wp.y.toFixed(2)})</span>
                    <div class="waypoint-actions">
                        <button class="icon-btn btn-primary" onclick="navigateToWaypoint(${index})">Go</button>
                        <button class="icon-btn btn-danger" onclick="deleteWaypoint(${index})">✕</button>
                    </div>
                `;
                list.appendChild(div);
            });
        }

        function addWaypointMode() {
            addingWaypoint = true;
            alert('Click on the map to add a waypoint');
        }

        canvas.addEventListener('click', (e) => {
            if (!addingWaypoint || !mapImage || !mapMetadata) return;

            const rect = canvas.getBoundingClientRect();
            const clickX = e.clientX - rect.left;
            const clickY = e.clientY - rect.top;

            // Convert to world coordinates
            const scale = Math.min(canvas.width / mapImage.width, canvas.height / mapImage.height) * 0.9;
            const offsetX = (canvas.width - mapImage.width * scale) / 2;
            const offsetY = (canvas.height - mapImage.height * scale) / 2;

            const mapX = (clickX - offsetX) / scale;
            const mapY = (clickY - offsetY) / scale;

            const resolution = mapMetadata.resolution;
            const originX = mapMetadata.origin[0];
            const originY = mapMetadata.origin[1];

            const worldX = mapX * resolution + originX;
            const worldY = (mapImage.height - mapY) * resolution + originY;

            waypoints.push({ x: worldX, y: worldY, theta: 0 });
            saveWaypoints();
            updateWaypointList();
            addingWaypoint = false;
        });

        async function saveWaypoints() {
            if (!currentMap) return;

            await fetch('/api/waypoints/save', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    map_name: currentMap,
                    waypoints: waypoints
                })
            });
        }

        function deleteWaypoint(index) {
            waypoints.splice(index, 1);
            saveWaypoints();
            updateWaypointList();
        }

        async function navigateToWaypoint(index) {
            const wp = waypoints[index];
            await fetch('/api/navigate', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ x: wp.x, y: wp.y, theta: wp.theta })
            });
        }

        async function checkNavigationStatus() {
            try {
                const response = await fetch('/api/navigate/status');
                const data = await response.json();
                isNavigating = data.status === 'active';
                document.getElementById('navCancelBtn').disabled = !isNavigating;
            } catch (error) {
                console.error('Error checking nav status:', error);
            }
        }

        async function cancelNavigation() {
            await fetch('/api/navigate/cancel', { method: 'POST' });
        }

        // SLAM Functions
        async function checkSlamStatus() {
            const response = await fetch('/api/slam/status');
            const data = await response.json();
            updateSlamUI(data.running);
            
            // Check periodically
            setTimeout(checkSlamStatus, 2000);
        }

        async function startSlam() {
            const response = await fetch('/api/slam/start', { method: 'POST' });
            const data = await response.json();
            if (data.success) {
                updateSlamUI(true);
            } else {
                alert('Failed to start SLAM: ' + (data.error || 'Unknown error'));
            }
        }

        async function stopSlam() {
            const response = await fetch('/api/slam/stop', { method: 'POST' });
            const data = await response.json();
            if (data.success) {
                updateSlamUI(false);
            }
        }

        function updateSlamUI(running) {
            document.getElementById('startSlamBtn').disabled = running;
            document.getElementById('stopSlamBtn').disabled = !running;
            document.getElementById('saveMapBtn').disabled = !running;
            document.getElementById('slamStatus').style.display = running ? 'block' : 'none';
        }

        function showSaveMapModal() {
            document.getElementById('saveMapModal').style.display = 'block';
        }

        function hideSaveMapModal() {
            document.getElementById('saveMapModal').style.display = 'none';
            document.getElementById('mapNameInput').value = '';
        }

        async function saveMap() {
            const mapName = document.getElementById('mapNameInput').value.trim();
            
            if (!mapName) {
                alert('Please enter a map name');
                return;
            }

            const response = await fetch('/api/map/save', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ name: mapName })
            });

            const data = await response.json();
            
            if (data.success) {
                alert('Map saved successfully!');
                hideSaveMapModal();
                loadMaps();
            } else {
                alert('Failed to save map: ' + (data.error || 'Unknown error'));
            }
        }

        // Close modal on outside click
        window.onclick = (event) => {
            const modal = document.getElementById('saveMapModal');
            if (event.target === modal) {
                hideSaveMapModal();
            }
        };
    </script>
</body>
</html>
"""

class WebNode(Node):
    def __init__(self):
        super().__init__('web_interface_node')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        
        # State
        self.robot_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.laser_scan = []
        self.map_metadata = None
        self.navigation_active = False
        
        self.get_logger().info('🚀 Web Interface Node initialized')
    
    def navigate_to_pose(self, x, y, theta=0):
        """Send navigation goal"""
        try:
            goal_msg = PoseStamped()
            goal_msg.header.frame_id = 'map'
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            
            goal_msg.pose.position.x = float(x)
            goal_msg.pose.position.y = float(y)
            goal_msg.pose.position.z = 0.0
            
            # Convert theta to quaternion
            qw = math.cos(theta / 2)
            qz = math.sin(theta / 2)
            
            goal_msg.pose.orientation.x = 0.0
            goal_msg.pose.orientation.y = 0.0
            goal_msg.pose.orientation.z = qz
            goal_msg.pose.orientation.w = qw
            
            self.goal_pub.publish(goal_msg)
            self.navigation_active = True
            
            self.get_logger().info(f'📍 Navigation goal sent: ({x:.2f}, {y:.2f})')
            return True
        except Exception as e:
            self.get_logger().error(f'❌ Navigation failed: {str(e)}')
            return False
    
    def cancel_navigation(self):
        """Cancel current navigation"""
        try:
            # Send zero velocity to stop
            self.publish_cmd_vel(0.0, 0.0)
            self.navigation_active = False
            self.get_logger().info('🛑 Navigation cancelled')
            return True
        except Exception as e:
            self.get_logger().error(f'❌ Cancel failed: {str(e)}')
            return False
    
    def odom_callback(self, msg):
        self.robot_pose['x'] = msg.pose.pose.position.x
        self.robot_pose['y'] = msg.pose.pose.position.y
        
        # Convert quaternion to euler
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        self.robot_pose['theta'] = math.atan2(siny_cosp, cosy_cosp)
    
    def laser_callback(self, msg):
        self.laser_scan = []
        step = max(1, len(msg.ranges) // 50)
        
        for i in range(0, len(msg.ranges), step):
            range_val = msg.ranges[i]
            
            if range_val < msg.range_min or range_val > msg.range_max or math.isinf(range_val) or math.isnan(range_val):
                continue
            
            angle = msg.angle_min + i * msg.angle_increment + self.robot_pose['theta']
            x = self.robot_pose['x'] + range_val * math.cos(angle)
            y = self.robot_pose['y'] + range_val * math.sin(angle)
            
            self.laser_scan.append({'x': x, 'y': y})
    
    def publish_cmd_vel(self, linear, angular):
        """Publish velocity command"""
        msg = Twist()
        msg.linear.x = float(linear)
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(angular)
        
        self.cmd_vel_pub.publish(msg)

web_node = None

@app.route('/api/waypoints/<map_name>')
def get_waypoints_api(map_name):
    return jsonify({'waypoints': load_waypoints(map_name)})

@app.route('/api/waypoints/save', methods=['POST'])
def save_waypoints_api():
    data = request.get_json()
    success = save_waypoints(data['map_name'], data['waypoints'])
    return jsonify({'success': success})

@app.route('/api/navigate', methods=['POST'])
def navigate():
    data = request.get_json()
    if web_node:
        success = web_node.navigate_to_pose(data['x'], data['y'], data.get('theta', 0))
        return jsonify({'success': success})
    return jsonify({'success': False})

@app.route('/api/navigate/cancel', methods=['POST'])
def cancel_nav():
    if web_node:
        return jsonify({'success': web_node.cancel_navigation()})
    return jsonify({'success': False})

@app.route('/api/navigate/status')
def nav_status():
    if web_node:
        status = 'active' if web_node.navigation_active else 'idle'
        return jsonify({'status': status})
    return jsonify({'status': 'idle'})
    
@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

@app.route('/api/data')
def get_data():
    if web_node:
        return jsonify({
            'robot_pose': web_node.robot_pose,
            'laser_scan': web_node.laser_scan[:100],
            'map_metadata': web_node.map_metadata
        })
    return jsonify({
        'robot_pose': {'x': 0, 'y': 0, 'theta': 0},
        'laser_scan': [],
        'map_metadata': None
    })

@app.route('/api/cmd_vel', methods=['POST'])
def cmd_vel():
    """Receive velocity commands from web interface"""
    try:
        data = request.get_json()
        linear = float(data.get('linear', 0.0))
        angular = float(data.get('angular', 0.0))
        
        if web_node:
            web_node.publish_cmd_vel(linear, angular)
            return jsonify({'success': True, 'linear': linear, 'angular': angular})
        else:
            return jsonify({'success': False, 'error': 'ROS node not initialized'}), 500
    except Exception as e:
        print(f"❌ Error in cmd_vel endpoint: {str(e)}")
        return jsonify({'success': False, 'error': str(e)}), 500

@app.route('/api/maps')
def get_maps():
    """Get list of available maps"""
    maps = get_available_maps()
    return jsonify({'maps': maps})

@app.route('/api/map/<map_name>')
def get_map(map_name):
    """Serve the map image file"""
    try:
        map_path = get_map_image_path(map_name)
        
        if not map_path or not os.path.exists(map_path):
            return jsonify({'error': 'Map not found'}), 404
        
        # Update map metadata for current map
        if web_node:
            web_node.map_metadata = load_map_yaml(map_name)
        
        # Serve PNG directly
        if map_path.endswith('.png'):
            return send_from_directory(
                os.path.dirname(map_path),
                os.path.basename(map_path),
                mimetype='image/png'
            )
        
        # Convert PGM to PNG
        elif map_path.endswith('.pgm'):
            png_path = map_path.replace('.pgm', '_temp.png')
            try:
                subprocess.run(['convert', map_path, png_path], check=True)
                return send_from_directory(
                    os.dirname(png_path),
                    os.path.basename(png_path),
                    mimetype='image/png'
                )
            except subprocess.CalledProcessError as e:
                return jsonify({'error': 'Failed to convert map'}), 500
        
        else:
            return jsonify({'error': 'Unsupported map format'}), 400
            
    except Exception as e:
        print(f"❌ Error serving map: {str(e)}")
        return jsonify({'error': str(e)}), 500

@app.route('/api/slam/status')
def slam_status():
    """Check SLAM status"""
    global slam_running
    return jsonify({'running': slam_running})

@app.route('/api/slam/start', methods=['POST'])
def start_slam():
    """Start SLAM mapping"""
    global slam_running
    
    if slam_running:
        return jsonify({'success': False, 'error': 'SLAM already running'})
    
    success = start_slam_mapping()
    return jsonify({'success': success})

@app.route('/api/slam/stop', methods=['POST'])
def stop_slam():
    """Stop SLAM mapping"""
    global slam_running
    
    if not slam_running:
        return jsonify({'success': False, 'error': 'SLAM not running'})
    
    success = stop_slam_mapping()
    return jsonify({'success': success})

@app.route('/api/map/save', methods=['POST'])
def save_map():
    """Save current map"""
    try:
        data = request.get_json()
        map_name = data.get('name', '')
        
        if not map_name:
            return jsonify({'success': False, 'error': 'Map name required'})
        
        # Validate map name
        if not map_name.replace('_', '').replace('-', '').isalnum():
            return jsonify({'success': False, 'error': 'Invalid map name'})
        
        success = save_current_map(map_name)
        
        if success:
            return jsonify({'success': True, 'map_name': map_name})
        else:
            return jsonify({'success': False, 'error': 'Failed to save map'})
    except Exception as e:
        print(f"❌ Error in save_map endpoint: {str(e)}")
        return jsonify({'success': False, 'error': str(e)}), 500

def ros_spin():
    rclpy.spin(web_node)

def main():
    global web_node
    
    # Ensure map directory exists
    os.makedirs(MAP_DIRECTORY, exist_ok=True)
    os.makedirs(WAYPOINTS_DIRECTORY, exist_ok=True)
    
    rclpy.init()
    web_node = WebNode()
    
    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()
    
    print("\n" + "="*60)
    print("🤖 Bumpy7 Enhanced Web Server - ANIMATED EDITION")
    print("="*60)
    
    print(f"\n📁 Map directory: {MAP_DIRECTORY}")
    print(f"📁 Waypoints directory: {WAYPOINTS_DIRECTORY}")
    
    available_maps = get_available_maps()
    if available_maps:
        print(f"📋 Available maps ({len(available_maps)}):")
        for map_info in available_maps:
            print(f"   • {map_info['name']} (created: {map_info['created']})")
    
    print("\n✨ Enhanced Features:")
    print("   • 🎨 Lively robot animations")
    print("   • ⚡ Smooth position transitions")
    print("   • 💫 Movement effects (bounce, spin, pulse)")
    print("   • 🎯 Real-time position tracking")
    print("   • 🗺️  Start/Stop SLAM mapping")
    print("   • 💾 Save maps with custom names")
    print("   • 📍 Waypoint navigation system")
    
    print("\n📡 ROS Topics:")
    print("   Subscribing:")
    print("   • /odom, /scan")
    print("   Publishing:")
    print(f"   • {CMD_VEL_TOPIC}")
    
    print("\n🌐 Access the web interface at:")
    print("   http://localhost:5000")
    local_ip = get_local_ip()
    print(f"   http://{local_ip}:5000")
    
    print("\n🔐 Login: bumpy7 / bumpykkr")
    print("="*60 + "\n")
    
    app.run(host='0.0.0.0', port=5000, debug=False)

if __name__ == '__main__':
    main()
