#!/usr/bin/env python3
"""
Car dashboard: a small web page (port 8080) that shows what the car is doing, in RC and autonomous mode alike.

Shows   controller / LiDAR / camera connected, mode, speed, steering angle, wall-ahead and reversing warnings,
        the LiDAR picture, the camera picture, and a Tuning tab that changes parameters while the car runs.
Reads   /drive_mode (joycon_teleop), /drive_cmd (what the car is told to do), /ego_racecar/odom (speed estimate),
        /scan (LiDAR), /camera/image/jpeg (person_detector), /mode_toggle_progress
Tunes   reactive_node, joycon_teleop and gpio_driver through their normal ROS parameter services.
"""
import json
import math
import os
import queue
import shutil
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32, Float32MultiArray, String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage, LaserScan
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters, GetParameters
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None

PORT = 8080
FRONT_CONE_DEG = 25          # "ahead" for the wall warning = this many degrees either side of straight

# Tuning tab. `node` says which ROS node owns the parameter; the current value is always read from that node.
# GET /params reads the live values, POST /set_param changes one. Keep names in sync with the nodes.
TUNABLE_PARAMS = {
    # ---- driving (joycon_teleop / gpio_driver) ----
    'rc_forward': {
        'node': 'joycon_teleop', 'group': 'Driving', 'type': 'double', 'label': 'Forward throttle at full stick (0 to 1)', 'default': 0.30,
        'desc': 'How much throttle you get with the stick fully forward in RC mode. Raise slowly.',
    },
    'rc_reverse': {
        'node': 'joycon_teleop', 'group': 'Driving', 'type': 'double', 'label': 'Reverse throttle at full stick (0 to 1)', 'default': 0.25,
        'desc': 'Same, for reverse.',
    },
    'throttle_slew_per_s': {
        'node': 'gpio_driver', 'group': 'Driving', 'type': 'double', 'label': 'Throttle ramp-up speed', 'default': 0.3,
        'desc': 'How fast the throttle may build up. 0.3 means about 1 second to reach 0.3 throttle. Lower = gentler start. Letting go always stops at once.',
    },
    'auto_max_throttle': {
        'node': 'joycon_teleop', 'group': 'Driving', 'type': 'double', 'label': 'Autonomous top throttle (0 to 1)', 'default': 0.25,
        'desc': 'The most throttle autonomous mode can use. Raise step by step as the car proves itself.',
    },
    'steer_deadzone': {
        'node': 'joycon_teleop', 'group': 'Driving', 'type': 'double', 'label': 'Steering stick dead zone', 'default': 0.12,
        'desc': 'How far you must move the stick before the wheels turn. Raise it if the wheels twitch with the stick centered.',
    },
    'throttle_deadzone': {
        'node': 'joycon_teleop', 'group': 'Driving', 'type': 'double', 'label': 'Throttle stick dead zone', 'default': 0.15,
        'desc': 'How far you must push the stick before the motor gets any throttle.',
    },
    # ---- steering linkage (gpio_driver) ----
    'steer_center_us': {
        'node': 'gpio_driver', 'group': 'Steering', 'type': 'double', 'label': 'Steering center (µs)', 'default': 1500.0,
        'desc': 'Servo pulse for straight wheels. Nudge it up or down until the car rolls straight.',
    },
    'steer_range_us': {
        'node': 'gpio_driver', 'group': 'Steering', 'type': 'double', 'label': 'Steering range (µs)', 'default': 200.0,
        'desc': 'How far from center the servo goes at full lock. Limited by the end stops set in the config file.',
    },
    'recenter_overshoot_left_us': {
        'node': 'gpio_driver', 'group': 'Steering', 'type': 'double', 'label': 'Recenter kick after a left turn (µs)', 'default': 40.0,
        'desc': 'When the wheels come back from a left turn they swing this far past center first, so the slack in the linkage is taken up. 0 = off.',
    },
    'recenter_overshoot_right_us': {
        'node': 'gpio_driver', 'group': 'Steering', 'type': 'double', 'label': 'Recenter kick after a right turn (µs)', 'default': 80.0,
        'desc': 'Same, after a right turn. Raise it if the wheels stay a little right after turning right.',
    },
    'recenter_overshoot_s': {
        'node': 'gpio_driver', 'group': 'Steering', 'type': 'double', 'label': 'Recenter kick hold time (s)', 'default': 0.15,
        'desc': 'How long the wheels stay at the kick position before settling on center.',
    },
    # ---- autonomous driving (reactive_node) ----
    'bubble_radius': {
        'node': 'reactive_node', 'group': 'Autonomous driving', 'type': 'double', 'label': 'Bubble radius (m)', 'default': 0.3,
        'desc': "Safety zone around the closest obstacle. The car won't aim through it. Higher = wider berth. Lower = cuts closer.",
    },
    'preprocess_conv_size': {
        'node': 'reactive_node', 'group': 'Autonomous driving', 'type': 'int', 'label': 'Smoothing window (readings)', 'default': 3,
        'desc': 'How many LiDAR readings get averaged together to smooth out noise. Higher = smoother but blurs small gaps.',
    },
    'max_lidar_range': {
        'node': 'reactive_node', 'group': 'Autonomous driving', 'type': 'double', 'label': 'How far it looks (m)', 'default': 3.5,
        'desc': 'Readings beyond this are capped. Higher = reacts sooner. Lower = only reacts to close things.',
    },
    'speed_max': {
        'node': 'reactive_node', 'group': 'Autonomous driving', 'type': 'double', 'label': 'Top speed it may ask for (m/s)', 'default': 15.0,
        'desc': 'The ceiling for the speed autonomous mode asks for. Actual speed is also limited by the autonomous top throttle above.',
    },
    'steering_gain': {
        'node': 'reactive_node', 'group': 'Autonomous driving', 'type': 'double', 'label': 'Steering strength', 'default': 0.38,
        'desc': 'How hard it turns toward its target. Higher = sharper (can overshoot). Lower = lazier.',
    },
    'max_steering': {
        'node': 'reactive_node', 'group': 'Autonomous driving', 'type': 'double', 'label': 'Max steering angle (deg)', 'default': 30.0,
        'desc': 'The steepest angle it may steer, either direction.',
    },
    'alpha_straight': {
        'node': 'reactive_node', 'group': 'Autonomous driving', 'type': 'double', 'label': 'Smoothing on straights (0 to 1)', 'default': 0.6,
        'desc': 'Steering smoothing when nearly straight. Higher = steadier, ignores tiny wiggles. Lower = twitchier.',
    },
    'alpha_curve': {
        'node': 'reactive_node', 'group': 'Autonomous driving', 'type': 'double', 'label': 'Smoothing in curves (0 to 1)', 'default': 0.6,
        'desc': 'Steering smoothing when turning sharply. Higher = smoother but slower to react. Lower = reacts fast, jumpier.',
    },
    'fov_deg': {
        'node': 'reactive_node', 'group': 'Autonomous driving', 'type': 'double', 'label': 'View width (deg each side)', 'default': 70.0,
        'desc': 'How far to each side it looks. Higher = sees further around bends but may turn early.',
    },
    'gap_aim_aggressiveness': {
        'node': 'reactive_node', 'group': 'Autonomous driving', 'type': 'double', 'label': 'Aim far or near (0 to 1)', 'default': 0.5,
        'desc': '0 = aim at the middle of the nearest gap (careful). 1 = aim at the farthest point (cuts corners).',
    },
    'straight_min_forward_frac': {
        'node': 'reactive_node', 'group': 'Autonomous driving', 'type': 'double', 'label': 'Clear distance to call it a straight', 'default': 0.75,
        'desc': 'Share of the look-ahead distance that must be clear to count as a straightaway and speed up.',
    },
    'straight_max_angle_deg': {
        'node': 'reactive_node', 'group': 'Autonomous driving', 'type': 'double', 'label': 'Steering allowed on a straight (deg)', 'default': 6.0,
        'desc': 'Steering must stay under this angle to count as driving straight.',
    },
    'straight_ema_alpha': {
        'node': 'reactive_node', 'group': 'Autonomous driving', 'type': 'double', 'label': 'How fast it notices a straight', 'default': 0.2,
        'desc': 'Higher = notices changes sooner. Lower = slower, steadier.',
    },
    'recovery_straight_time': {
        'node': 'reactive_node', 'group': 'Autonomous driving', 'type': 'double', 'label': 'Reverse straight for (s)', 'default': 0.8,
        'desc': 'When stuck, how long it backs up in a straight line before steering away from the wall.',
    },
    'recovery_clear_dist': {
        'node': 'reactive_node', 'group': 'Autonomous driving', 'type': 'double', 'label': 'Space needed to go again (m)', 'default': 1.0,
        'desc': 'Free space needed on both sides before it resumes driving after backing up.',
    },
    'recovery_max_time': {
        'node': 'reactive_node', 'group': 'Autonomous driving', 'type': 'double', 'label': 'Longest it will back up (s)', 'default': 6.0,
        'desc': 'After this long reversing it gives up and drives on, so it can never back up forever.',
    },
}
PARAM_NODES = sorted({m['node'] for m in TUNABLE_PARAMS.values()})

# Set once CarDashboard() is constructed in main(): Handler is created per request with no other way to reach it.
dashboard_node = None


def find_dashboard_html():
    """Look in the installed share dir first, then fall back to the source tree."""
    if get_package_share_directory is not None:
        try:
            share = get_package_share_directory('follow_the_gap')
            candidate = os.path.join(share, 'dashboard', 'car_dashboard.html')
            if os.path.exists(candidate):
                return candidate
        except Exception:
            pass
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(here, '..', 'dashboard', 'car_dashboard.html')


HTML_PATH = find_dashboard_html()


class DashboardState:
    """Shared between the ROS thread and the HTTP server threads."""

    def __init__(self):
        self.lock = threading.Lock()
        self.clients = []
        self.latest = {
            'controller': False, 'drive_mode': 'NO_TELEOP', 'toggle_progress': 0.0,
            'lidar_ok': False, 'scan': [], 'front_cm': -1,
            'speed': 0.0, 'steering_deg': 0.0, 'throttle': 0.0, 'reversing': False,
            'camera_age': -1.0, 't': 0.0,
        }

    def update(self, partial):
        with self.lock:
            self.latest = {**self.latest, **partial}
            snapshot = dict(self.latest)
            for q in self.clients:
                q.put(snapshot)

    def register(self):
        q = queue.Queue()
        with self.lock:
            self.clients.append(q)
        return q

    def unregister(self, q):
        with self.lock:
            if q in self.clients:
                self.clients.remove(q)


state = DashboardState()


class CameraState:
    """Newest camera JPEG from person_detector, shared with the HTTP threads that stream it."""

    def __init__(self):
        self.cond = threading.Condition()
        self.jpeg = None
        self.seq = 0
        self.t = 0.0


camera = CameraState()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass  # keep the terminal quiet

    def do_GET(self):
        if self.path in ('/', '/index.html'):
            self._serve_html()
        elif self.path == '/stream':
            self._serve_stream()
        elif self.path == '/params':
            self._serve_params()
        elif self.path.startswith('/camera.mjpg'):
            self._serve_camera_stream()
        elif self.path.startswith('/camera.jpg'):
            self._serve_camera_still()
        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        if self.path == '/set_param':
            self._handle_set_param()
        else:
            self.send_response(404)
            self.end_headers()

    def _json_response(self, code, obj):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _read_json_body(self):
        length = int(self.headers.get('Content-Length', 0) or 0)
        raw = self.rfile.read(length) if length > 0 else b'{}'
        return json.loads(raw or b'{}')

    def _serve_params(self):
        live = dashboard_node.get_all_params() if dashboard_node else {}
        out = {}
        for name, meta in TUNABLE_PARAMS.items():
            value = live.get(name)
            out[name] = {
                'label': meta['label'], 'desc': meta.get('desc', ''), 'type': meta['type'],
                'group': meta['group'], 'node': meta['node'], 'default': meta.get('default'),
                'value': value if value is not None else meta.get('default'),
                'live': value is not None,   # False = that node did not answer, so the box shows the start value
            }
        self._json_response(200, out)

    def _handle_set_param(self):
        try:
            payload = self._read_json_body()
            name = payload['name']
            value = payload['value']
        except Exception:
            self._json_response(400, {'ok': False, 'reason': 'bad request body'})
            return
        meta = TUNABLE_PARAMS.get(name)
        if meta is None:
            self._json_response(400, {'ok': False, 'reason': f'unknown parameter: {name}'})
            return
        if dashboard_node is None:
            self._json_response(503, {'ok': False, 'reason': 'dashboard not ready'})
            return
        ok, reason = dashboard_node.set_param(meta['node'], name, value, meta['type'])
        self._json_response(200 if ok else 500, {'ok': ok, 'reason': reason})

    def _serve_html(self):
        try:
            with open(HTML_PATH, 'rb') as f:
                body = f.read()
        except FileNotFoundError:
            self.send_response(500)
            self.end_headers()
            self.wfile.write(b'car_dashboard.html not found at ' + HTML_PATH.encode())
            return
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Cache-Control', 'no-cache')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _serve_camera_still(self):
        with camera.cond:
            jpeg = camera.jpeg
        if jpeg is None:
            self._json_response(503, {'ok': False, 'reason': 'no camera frame yet'})
            return
        self.send_response(200)
        self.send_header('Content-Type', 'image/jpeg')
        self.send_header('Cache-Control', 'no-cache')
        self.send_header('Content-Length', str(len(jpeg)))
        self.end_headers()
        self.wfile.write(jpeg)

    def _serve_camera_stream(self):
        """MJPEG: the browser shows it with a plain <img src="/camera.mjpg">."""
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.send_header('Cache-Control', 'no-cache')
        self.send_header('Connection', 'close')
        self.end_headers()
        last = -1
        try:
            while True:
                with camera.cond:
                    camera.cond.wait_for(lambda: camera.seq != last, timeout=5.0)
                    jpeg, last = camera.jpeg, camera.seq
                if jpeg is None:
                    continue
                self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n' % len(jpeg))
                self.wfile.write(jpeg)
                self.wfile.write(b'\r\n')
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _serve_stream(self):
        self.send_response(200)
        self.send_header('Content-Type', 'text/event-stream')
        self.send_header('Cache-Control', 'no-cache')
        self.send_header('Connection', 'keep-alive')
        self.end_headers()

        q = state.register()
        try:
            self.wfile.write(f'data: {json.dumps(state.latest)}\n\n'.encode())
            self.wfile.flush()
            while True:
                try:
                    data = q.get(timeout=15)
                    self.wfile.write(f'data: {json.dumps(data)}\n\n'.encode())
                    self.wfile.flush()
                except queue.Empty:
                    self.wfile.write(b': keep-alive\n\n')
                    self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            pass
        finally:
            state.unregister(q)


class CarDashboard(Node):
    def __init__(self):
        super().__init__('car_dashboard')
        self.declare_parameter('max_steer_deg', 30.0)        # steering angle shown at full stick
        self.declare_parameter('lidar_front_offset_deg', 0.0)  # turn the LiDAR picture if it is mounted rotated
        self.max_steer_deg = float(self.get_parameter('max_steer_deg').value)
        self.front_offset = math.radians(float(self.get_parameter('lidar_front_offset_deg').value))

        self.create_subscription(String, '/drive_mode', self._on_mode, 10)
        self.create_subscription(Float32, '/mode_toggle_progress', self._on_progress, 10)
        self.create_subscription(Float32MultiArray, '/drive_cmd', self._on_cmd, 10)
        self.create_subscription(Odometry, '/ego_racecar/odom', self._on_odom, 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(CompressedImage, '/camera/image/jpeg', self._on_camera, 1)

        self._mode, self._mode_t = 'NO_TELEOP', 0.0
        self._progress = 0.0
        self._steer, self._throttle, self._cmd_t = 0.0, 0.0, 0.0
        self._speed, self._odom_t = 0.0, 0.0
        self._scan_t, self._scan_proc_t = 0.0, 0.0
        self._bins, self._front_cm = [], -1

        self.create_timer(0.1, self._push)   # one snapshot to the browsers, 10 times a second

        # live tuning: each node's own standard parameter services
        self._param_clients = {
            n: (self.create_client(GetParameters, f'/{n}/get_parameters'),
                self.create_client(SetParameters, f'/{n}/set_parameters'))
            for n in PARAM_NODES
        }
        self.get_logger().info('dashboard ready')

    # ---------------- ROS callbacks: only remember the latest value ----------------
    def _on_mode(self, msg):
        self._mode, self._mode_t = msg.data, time.time()

    def _on_progress(self, msg):
        self._progress = float(msg.data)

    def _on_cmd(self, msg):
        if len(msg.data) >= 2:
            self._steer, self._throttle, self._cmd_t = float(msg.data[0]), float(msg.data[1]), time.time()

    def _on_odom(self, msg):
        self._speed, self._odom_t = float(msg.twist.twist.linear.x), time.time()

    def _on_camera(self, msg):
        with camera.cond:
            camera.jpeg = bytes(msg.data)
            camera.seq += 1
            camera.t = time.time()
            camera.cond.notify_all()

    def _on_scan(self, msg):
        now = time.time()
        self._scan_t = now
        if now - self._scan_proc_t < 0.2:   # the picture only needs ~5 updates a second
            return
        self._scan_proc_t = now
        bins = [0] * 360                    # one cm value per degree; 0 = no reading. Degree 0 = straight ahead, counting to the left
        start = msg.angle_min + self.front_offset
        rmin = max(float(msg.range_min), 0.05)
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < rmin or r > 60.0:
                continue
            deg = int(round(math.degrees(start + i * msg.angle_increment))) % 360
            cm = max(1, int(r * 100))
            if bins[deg] == 0 or cm < bins[deg]:
                bins[deg] = cm
        self._bins = bins
        ahead = [bins[d] for d in list(range(0, FRONT_CONE_DEG + 1)) + list(range(360 - FRONT_CONE_DEG, 360)) if bins[d] > 0]
        self._front_cm = min(ahead) if ahead else -1

    # ---------------- one snapshot for the browsers ----------------
    def _push(self):
        now = time.time()
        mode = self._mode if now - self._mode_t < 2.0 else 'NO_TELEOP'
        cmd_fresh = now - self._cmd_t < 1.0
        throttle = self._throttle if cmd_fresh else 0.0
        steering = self._steer if cmd_fresh else 0.0
        speed = self._speed if now - self._odom_t < 1.0 else 0.0
        lidar_ok = now - self._scan_t < 2.0
        cam_age = round(now - camera.t, 1) if camera.t else -1.0
        state.update({
            'controller': mode not in ('NO_CONTROLLER', 'NO_TELEOP'),
            'drive_mode': mode,
            'toggle_progress': self._progress,
            'lidar_ok': lidar_ok,
            'scan': self._bins if lidar_ok else [],
            'front_cm': self._front_cm if lidar_ok else -1,
            'speed': round(speed, 2),
            'steering_deg': round(steering * self.max_steer_deg, 1),   # + = left
            'throttle': round(throttle, 2),
            'reversing': speed < -0.05 or throttle < -0.01,
            'camera_age': cam_age,
            't': now,
        })

    # ---------------- live tuning ----------------
    def _call_service_sync(self, client, request, timeout=2.0):
        """Bridge a ROS service call to a blocking HTTP handler thread."""
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        future = client.call_async(request)
        done = threading.Event()
        holder = {}

        def _on_done(f):
            holder['result'] = f.result()
            done.set()

        future.add_done_callback(_on_done)
        if not done.wait(timeout):
            return None
        return holder.get('result')

    def get_all_params(self):
        """{name: current value} for every tunable parameter whose node answers."""
        out = {}
        for node in PARAM_NODES:
            names = [n for n, m in TUNABLE_PARAMS.items() if m['node'] == node]
            req = GetParameters.Request()
            req.names = names
            resp = self._call_service_sync(self._param_clients[node][0], req, timeout=1.0)
            if resp is None:
                continue
            for name, pv in zip(names, resp.values):
                if pv.type == ParameterType.PARAMETER_DOUBLE:
                    out[name] = pv.double_value
                elif pv.type == ParameterType.PARAMETER_INTEGER:
                    out[name] = pv.integer_value
        return out

    def set_param(self, node, name, value, ptype):
        p = Parameter()
        p.name = name
        pv = ParameterValue()
        if ptype == 'int':
            pv.type = ParameterType.PARAMETER_INTEGER
            pv.integer_value = int(value)
        else:
            pv.type = ParameterType.PARAMETER_DOUBLE
            pv.double_value = float(value)
        p.value = pv
        req = SetParameters.Request()
        req.parameters = [p]
        resp = self._call_service_sync(self._param_clients[node][1], req)
        if resp is None:
            return False, f'{node} did not answer (is it running?)'
        if not resp.results:
            return False, 'no result returned'
        result = resp.results[0]
        return result.successful, result.reason


def start_http_server():
    server = ThreadingHTTPServer(('0.0.0.0', PORT), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server


def open_dashboard(url):
    """Open a browser on url without blocking (only useful where a screen is attached)."""
    os.environ.setdefault('DISPLAY', ':99')
    for browser in ('epiphany-browser', 'epiphany', 'chromium', 'firefox', 'xdg-open'):
        path = shutil.which(browser)
        if not path:
            continue
        try:
            subprocess.Popen([path, url], start_new_session=True,
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            return True
        except OSError:
            continue
    return False


def main(args=None):
    global dashboard_node
    rclpy.init(args=args)

    start_http_server()
    url = f'http://localhost:{PORT}'
    print('\n' + '=' * 60)
    print('  CAR DASHBOARD')
    print(f'  {url}   (from another computer: http://<car address>:{PORT})')
    print('=' * 60 + '\n')

    node = CarDashboard()
    dashboard_node = node
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
