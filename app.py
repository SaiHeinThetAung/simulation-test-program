import threading
import time
import random
import math
from datetime import datetime
from flask import Flask, render_template, jsonify, request
from flask_socketio import SocketIO, emit
from pymavlink import mavutil
from geopy.distance import geodesic
from geopy.point import Point
import logging
import socket
# ---- Logging ----
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)
# ---- Flask/socketio ----
app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app, cors_allowed_origins="*")
DEFAULT_HOST = '192.168.0.197'
DEFAULT_PORTS = []
UPDATE_INTERVAL = 0.5 # Modified to 0.5 seconds per the request for sending drone data
TRAVEL_TIME = 1000
TRAVEL_DISTANCE = 5000
# ---- Asian locations ----
asian_locations = [
    {"country": "Afghanistan", "capital": "Kabul", "lat": 34.51666667, "lon": 69.183333},
    {"country": "Armenia", "capital": "Yerevan", "lat": 40.16666667, "lon": 44.5},
    {"country": "Azerbaijan", "capital": "Baku", "lat": 40.38333333, "lon": 49.866667},
    {"country": "Bahrain", "capital": "Manama", "lat": 26.23333333, "lon": 50.566667},
    {"country": "Bangladesh", "capital": "Dhaka", "lat": 23.71666667, "lon": 90.4},
    {"country": "Bhutan", "capital": "Thimphu", "lat": 27.46666667, "lon": 89.633333},
    {"country": "Brunei Darussalam", "capital": "Bandar Seri Begawan", "lat": 4.883333333, "lon": 114.933333},
    {"country": "Cambodia", "capital": "Phnom Penh", "lat": 11.55, "lon": 104.916667},
    {"country": "China", "capital": "Beijing", "lat": 39.91666667, "lon": 116.383333},
    {"country": "Georgia", "capital": "Tbilisi", "lat": 41.68333333, "lon": 44.833333},
    {"country": "India", "capital": "New Delhi", "lat": 28.6, "lon": 77.2},
    {"country": "Indonesia", "capital": "Jakarta", "lat": -6.166666667, "lon": 106.816667},
    {"country": "Iran", "capital": "Tehran", "lat": 35.7, "lon": 51.416667},
    {"country": "Iraq", "capital": "Baghdad", "lat": 33.33333333, "lon": 44.4},
    {"country": "Israel", "capital": "Jerusalem", "lat": 31.76666667, "lon": 35.233333},
    {"country": "Japan", "capital": "Tokyo", "lat": 35.68333333, "lon": 139.75},
    {"country": "Jordan", "capital": "Amman", "lat": 31.95, "lon": 35.933333},
    {"country": "Kazakhstan", "capital": "Astana", "lat": 51.16666667, "lon": 71.416667},
    {"country": "Kuwait", "capital": "Kuwait City", "lat": 29.36666667, "lon": 47.966667},
    {"country": "Kyrgyzstan", "capital": "Bishkek", "lat": 42.86666667, "lon": 74.6},
    {"country": "Laos", "capital": "Vientiane", "lat": 17.96666667, "lon": 102.6},
    {"country": "Lebanon", "capital": "Beirut", "lat": 33.86666667, "lon": 35.5},
    {"country": "Malaysia", "capital": "Kuala Lumpur", "lat": 3.166666667, "lon": 101.7},
    {"country": "Maldives", "capital": "Male", "lat": 4.166666667, "lon": 73.5},
    {"country": "Mongolia", "capital": "Ulaanbaatar", "lat": 47.91666667, "lon": 106.916667},
    {"country": "Myanmar", "capital": "Yangon", "lat": 16.8, "lon": 96.15},
    {"country": "Nepal", "capital": "Kathmandu", "lat": 27.71666667, "lon": 85.316667},
    {"country": "North Korea", "capital": "Pyongyang", "lat": 39.01666667, "lon": 125.75},
    {"country": "Oman", "capital": "Muscat", "lat": 23.61666667, "lon": 58.583333},
    {"country": "Pakistan", "capital": "Islamabad", "lat": 33.68333333, "lon": 73.05},
    {"country": "Philippines", "capital": "Manila", "lat": 14.6, "lon": 120.966667},
    {"country": "Qatar", "capital": "Doha", "lat": 25.28333333, "lon": 51.533333},
    {"country": "Russia", "capital": "Moscow", "lat": 55.75, "lon": 37.6},
    {"country": "Saudi Arabia", "capital": "Riyadh", "lat": 24.65, "lon": 46.7},
    {"country": "Singapore", "capital": "Singapore", "lat": 1.283333333, "lon": 103.85},
    {"country": "South Korea", "capital": "Seoul", "lat": 37.55, "lon": 126.983333},
    {"country": "Sri Lanka", "capital": "Colombo", "lat": 6.916666667, "lon": 79.833333},
    {"country": "Syria", "capital": "Damascus", "lat": 33.5, "lon": 36.3},
    {"country": "Taiwan", "capital": "Taipei", "lat": 25.03333333, "lon": 121.516667},
    {"country": "Tajikistan", "capital": "Dushanbe", "lat": 38.55, "lon": 68.766667},
    {"country": "Thailand", "capital": "Bangkok", "lat": 13.75, "lon": 100.516667},
    {"country": "Turkey", "capital": "Ankara", "lat": 39.93333333, "lon": 32.866667},
    {"country": "Turkmenistan", "capital": "Ashgabat", "lat": 37.95, "lon": 58.383333},
    {"country": "United Arab Emirates", "capital": "Abu Dhabi", "lat": 24.46666667, "lon": 54.366667},
    {"country": "Uzbekistan", "capital": "Tashkent", "lat": 41.31666667, "lon": 69.25},
    {"country": "Vietnam", "capital": "Hanoi", "lat": 21.03333333, "lon": 105.85},
    {"country": "Yemen", "capital": "Sanaa", "lat": 15.35, "lon": 44.2},
    {"country": "Palestine", "capital": "Jerusalem", "lat": 31.76666667, "lon": 35.233333},
    {"country": "East Timor", "capital": "Dili", "lat": -8.583333333, "lon": 125.6}
]
# ---- Global state ----
telemetry_data = {}
lock = threading.Lock()
_sim_threads = [] # list of threading.Thread objects
_stop_events = [] # list of threading.Event objects
_sim_config = {
    "host": DEFAULT_HOST,
    "ports": DEFAULT_PORTS
}
# ---- Drone simulator class ----
class DroneSimulator:
    def __init__(self, drone_id, port, host, source_lat, source_lon, country, update_interval=UPDATE_INTERVAL):
        self.drone_id = drone_id
        self.port = port
        self.host = host
        self.country = country
        self.update_interval = update_interval
        # system_id = last two digits of port (01-99 → 1-99, 00 → 100)
        self.system_id = self._get_system_id_from_port()
        self.connection = None
        self.source_lat = source_lat + random.uniform(-0.01, 0.01)
        self.source_lon = source_lon + random.uniform(-0.01, 0.01)
        self.bearing = random.uniform(0, 360)
        source = Point(latitude=self.source_lat, longitude=self.source_lon)
        destination = geodesic(meters=TRAVEL_DISTANCE).destination(source, self.bearing)
        self.dest_lat = destination.latitude
        self.dest_lon = destination.longitude
        self.current_lat = self.source_lat
        self.current_lon = self.source_lon
        self.current_alt = 0.0
        self.ground_speed = TRAVEL_DISTANCE / TRAVEL_TIME # m/s
        self.vertical_speed = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = self.bearing
        self.battery_voltage = 12.6
        self.battery_current = 0.0
        self.ch3out = 1000
        self.start_time = time.time()
        self.wp_dist = TRAVEL_DISTANCE
        self.airspeed = self.ground_speed * 1.2
        self.wind_speed = random.uniform(0.0, 5.0)
        self.gps_hdop = random.uniform(1.0, 2.0)
        self.waypoints = self._generate_dummy_waypoints()
        self.mission_count = len(self.waypoints)
        self.dist_traveled = 0.0
        self.dist_to_home = 0.0
    def _get_system_id_from_port(self):
        """Extract last two digits of port and convert to system_id (1-100).
        Examples: 1401 → 1, 1415 → 15, 1499 → 99, 1500 → 100"""
        last_two = self.port % 100
        if last_two == 0:
            return 100 # Avoid sysid 0 (invalid in MAVLink), use 100 instead
        return last_two
    def _generate_dummy_waypoints(self):
        waypoints = [
            {'seq': 0, 'lat': self.source_lat, 'lon': self.source_lon, 'alt': 30.0},
            {'seq': 1, 'lat': self.dest_lat, 'lon': self.dest_lon, 'alt': 50.0}
        ]
        return waypoints
    def connect_udp(self):
        for attempt in range(3):
            try:
                self.connection = mavutil.mavlink_connection(
                    f'udpout:{self.host}:{self.port}',
                    source_system=self.system_id, # Now 1–100 based on port
                    source_component=1,
                    dialect='common'
                )
                logger.info(f"[Drone {self.drone_id}] Port {self.port} → SysID {self.system_id} → sending to {self.host}:{self.port}")
                return
            except Exception as e:
                logger.warning(f"[Drone {self.drone_id}] connect attempt {attempt+1} failed: {e}")
                self.connection = None
                time.sleep(1)
        logger.error(f"[Drone {self.drone_id}] failed to configure UDP after attempts")
    def send_heartbeat(self):
        try:
            if self.connection:
                self.connection.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_QUADROTOR, # Changed to more realistic type
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0
                )
        except Exception as e:
            logger.debug(f"[Drone {self.drone_id}] heartbeat error: {e}")
            self.connection = None
    def send_mission_count(self):
        try:
            if self.connection:
                self.connection.mav.mission_count_send(1, 1, self.mission_count)
        except Exception:
            self.connection = None
    def send_mission_item_int(self, waypoint):
        try:
            if self.connection:
                self.connection.mav.mission_item_int_send(
                    1, 1, waypoint['seq'],
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    0, 0, 0, 0, 0, waypoint['alt'],
                    int(waypoint['lat'] * 1e7), int(waypoint['lon'] * 1e7), 0
                )
        except Exception:
            self.connection = None
    def send_global_position_int(self):
        try:
            if self.connection:
                ground_speed_cms = int(min(max(self.ground_speed * 100, -32767), 32767))
                vertical_speed_cms = int(min(max(self.vertical_speed * 100, -32767), 32767))
                heading_cdeg = int(min(max(self.bearing * 100, 0), 35999))
                self.connection.mav.global_position_int_send(
                    0, int(self.current_lat * 1e7), int(self.current_lon * 1e7),
                    int(self.current_alt * 1000), int(self.current_alt * 1000),
                    ground_speed_cms, ground_speed_cms, vertical_speed_cms, heading_cdeg
                )
        except Exception:
            self.connection = None
    def send_nav_controller_output(self):
        try:
            if self.connection:
                self.connection.mav.nav_controller_output_send(
                    0, 0, 0, int(self.wp_dist), 0, 0, 0, 0, 0
                )
        except Exception:
            self.connection = None
    def send_vfr_hud(self):
        try:
            if self.connection:
                throttle_percent = int(min(max(((self.ch3out - 1000) / 1000.0) * 100, 0), 100))
                self.connection.mav.vfr_hud_send(
                    float(self.airspeed), float(self.ground_speed), int(self.bearing),
                    throttle_percent, float(self.current_alt), float(self.vertical_speed)
                )
        except Exception:
            self.connection = None
    def send_attitude(self):
        try:
            if self.connection:
                self.connection.mav.attitude_send(
                    0, float(math.radians(self.roll)), float(math.radians(self.pitch)),
                    float(math.radians(self.yaw)), 0, 0, 0
                )
        except Exception:
            self.connection = None
    def send_sys_status(self):
        try:
            if self.connection:
                self.connection.mav.sys_status_send(
                    0, 0, 0, 0,
                    int(self.battery_voltage * 1000),
                    int(self.battery_current * 100),
                    -1, 0, 0, 0, 0, 0, 0
                )
        except Exception:
            self.connection = None
    def send_servo_output_raw(self):
        try:
            if self.connection:
                self.connection.mav.servo_output_raw_send(
                    0, 0, 1000, 1000, self.ch3out, 1000, 1000, 1000, 1000, 1000, 0
                )
        except Exception:
            self.connection = None
    def update_telemetry(self):
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        fraction = (elapsed_time % TRAVEL_TIME) / TRAVEL_TIME
        if fraction >= 1.0:
            self.start_time = current_time
            fraction = 0.0
            self.current_lat = self.source_lat
            self.current_lon = self.source_lon
            self.dist_traveled = 0.0
            self.ch3out = 1000
        source = Point(latitude=self.source_lat, longitude=self.source_lon)
        current = geodesic(meters=TRAVEL_DISTANCE * fraction).destination(source, self.bearing)
        self.current_lat = current.latitude
        self.current_lon = current.longitude
        if self.current_alt < 50.0:
            self.vertical_speed = 1.0
            self.current_alt += self.vertical_speed * self.update_interval
            if self.current_alt > 50.0:
                self.current_alt = 50.0
        else:
            self.vertical_speed = 0.0
        if self.current_alt > 0.9:
            self.ch3out = random.randint(1500, 2000)
        else:
            self.ch3out = 1000
        self.dist_traveled = TRAVEL_DISTANCE * fraction
        self.dist_to_home = geodesic((self.current_lat, self.current_lon), (self.source_lat, self.source_lon)).meters
        self.wp_dist = TRAVEL_DISTANCE - self.dist_traveled
        self.roll = random.uniform(-10, 10)
        self.pitch = random.uniform(-5, 5)
        self.yaw = self.bearing
        self.wind_speed = random.uniform(0.0, 5.0)
        self.gps_hdop = random.uniform(1.0, 2.0)
        ch3percent = ((self.ch3out - 1000) / 1000.0) * 100
        tot = self.wp_dist / max(self.ground_speed, 0.1) if self.ground_speed > 0 else 0
        toh = self.dist_to_home / max(self.ground_speed, 0.1) if self.ground_speed > 0 else 0
        telemetry = {
            'port': self.port,
            'country': self.country,
            'GCS_IP': self.host,
            'system_id': self.system_id, # Now exactly matches last two digits (1–100)
            'flight_status': 1 if self.ch3out > 1050 else 0,
            'auto_time': 0,
            'throttle_active': self.ch3out > 1050,
            'lat': round(self.current_lat, 7),
            'lon': round(self.current_lon, 7),
            'alt': round(self.current_alt, 3),
            'dist_traveled': round(self.dist_traveled, 3),
            'wp_dist': round(self.wp_dist, 3),
            'heading': round(self.bearing, 0),
            'target_heading': int(self.bearing),
            'previous_heading': round(self.bearing - 10, 0) % 360,
            'dist_to_home': round(self.dist_to_home, 3),
            'vertical_speed': round(self.vertical_speed, 6),
            'ground_speed': round(self.ground_speed, 6),
            'wind_vel': round(self.wind_speed, 1),
            'airspeed': round(self.airspeed, 6),
            'gps_hdop': round(self.gps_hdop, 2),
            'roll': round(self.roll, 2),
            'pitch': round(self.pitch, 2),
            'yaw': round(self.yaw, 1),
            'ch3percent': round(ch3percent, 1),
            'ch9out': 0,
            'tot': round(tot, 2),
            'toh': round(toh, 2),
            'time_in_air': 0,
            'throttle_time_in_air': 0,
            'ch10out': 0,
            'ch11out': 0,
            'ch12out': 0,
            'battery_voltage': int(self.battery_voltage),
            'battery_current': round(self.battery_current, 1),
            'waypoints_count': self.mission_count,
            'home_location': {'lat': round(self.source_lat, 7), 'lon': round(self.source_lon, 7)},
            'waypoints': self.waypoints,
            'ch3out': self.ch3out,
            'timestamp': datetime.now().isoformat(),
            'flight_count': 1
        }
        with lock:
            telemetry_data[self.port] = telemetry
        try:
            socketio.emit('telemetry_update', telemetry_data)
        except Exception:
            pass
        return telemetry
    def run(self, stop_event: threading.Event):
        self.connect_udp()
        if self.connection:
            try:
                self.send_heartbeat()
                self.send_mission_count()
                time.sleep(0.05)
                for wp in self.waypoints:
                    self.send_mission_item_int(wp)
                    time.sleep(0.02)
            except Exception as e:
                logger.debug(f"[Drone {self.drone_id}] initial send error: {e}")
        while not stop_event.is_set():
            self.update_telemetry()
            if self.connection is None:
                self.connect_udp()
            if self.connection:
                try:
                    self.send_heartbeat()
                    self.send_global_position_int()
                    self.send_nav_controller_output()
                    self.send_vfr_hud()
                    self.send_attitude()
                    self.send_sys_status()
                    self.send_servo_output_raw()
                except Exception as e:
                    logger.debug(f"[Drone {self.drone_id}] send failed: {e}")
                    self.connection = None
            sleep_acc = 0.0
            while sleep_acc < self.update_interval and not stop_event.is_set():
                time.sleep(0.1)
                sleep_acc += 0.1
        try:
            if self.connection:
                self.connection.close()
                logger.info(f"[Drone {self.drone_id}] connection closed")
        except Exception as e:
            logger.debug(f"[Drone {self.drone_id}] error closing connection: {e}")
# ---- Simulator control helpers ----
def start_simulators(host, ports):
    global _sim_threads, _stop_events, telemetry_data
    stop_events = []
    threads = []
    num_drones = len(ports)
    logger.info(f"Spawning {num_drones} drone simulators -> {host}:{', '.join(map(str, sorted(ports)))}")
    with lock:
        telemetry_data.clear()
    sorted_ports = sorted(ports)
    num_locations = len(asian_locations)
    for i, port in enumerate(sorted_ports):
        loc_index = (port - 15001) % num_locations  # Map port 15001 to 0, 15002 to 1, etc.
        loc = asian_locations[loc_index]
        drone = DroneSimulator(i + 1, port, host, source_lat=loc['lat'], source_lon=loc['lon'], country=loc['country'])
        stop_event = threading.Event()
        t = threading.Thread(target=drone.run, args=(stop_event,), daemon=True)
        t.start()
        threads.append(t)
        stop_events.append(stop_event)
        time.sleep(0.05)
    _sim_threads = threads
    _stop_events = stop_events
    return threads, stop_events
def stop_simulators():
    global _sim_threads, _stop_events, telemetry_data
    logger.info("Stopping simulators...")
   
    if not _sim_threads:
        logger.info("No simulators are currently running")
        return
   
    for ev in _stop_events:
        ev.set()
    for t in _sim_threads:
        t.join(timeout=2.0)
        if t.is_alive():
            logger.warning(f"Thread {t.name} did not stop gracefully")
    _sim_threads = []
    _stop_events = []
    with lock:
        telemetry_data.clear()
   
    logger.info("All simulators stopped")
# ---- Flask routes ----
@app.route('/')
def index():
    return render_template('index.html')
@app.route('/start', methods=['POST'])
def start_route():
    data = request.get_json(force=True)
    if not data:
        return jsonify({"status": "error", "message": "No JSON body provided"}), 400
    host = data.get('host') or DEFAULT_HOST
    ports = data.get('ports')
    if not isinstance(ports, list) or len(ports) == 0 or not all(isinstance(p, int) for p in ports):
        return jsonify({"status": "error", "message": "ports must be a non-empty list of integers"}), 400
    # Deduplicate ports if needed
    ports = list(set(ports))
    num_drones = len(ports)
    stop_simulators()
    _sim_config['host'] = host
    _sim_config['ports'] = ports
    start_simulators(host, ports)
    socketio.emit('telemetry_update', {})
    return jsonify({
        "status": "started",
        "host": host,
        "ports": ports,
        "num_drones": num_drones
    })
@app.route('/stop', methods=['POST'])
def stop_route():
    logger.info("Stop route called from UI")
    stop_simulators()
    with lock:
        telemetry_data.clear()
    socketio.emit('telemetry_update', {})
    return jsonify({"status": "stopped", "message": "All simulators stopped"})
@app.route('/debug', methods=['GET'])
def debug_route():
    with lock:
        return jsonify(dict(telemetry_data))
@app.route('/status', methods=['GET'])
def status_route():
    with lock:
        num_drones = len(telemetry_data)
    return jsonify({
        "status": "running" if _sim_threads else "stopped",
        "num_drones": num_drones,
        "threads_alive": len([t for t in _sim_threads if t.is_alive()]) if _sim_threads else 0
    })
@socketio.on('connect')
def handle_connect():
    logger.info("Client connected to WebSocket")
    with lock:
        socketio.emit('telemetry_update', telemetry_data)
if __name__ == '__main__':
    logger.info("Starting Flask + Socket.IO server on http://0.0.0.0:5000")
    socketio.run(app, host='0.0.0.0', port=5000, debug=False)