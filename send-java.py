import threading
import time
import random
import math
from datetime import datetime
from flask import Flask, render_template, jsonify
from flask_socketio import SocketIO
from pymavlink import mavutil
from geopy.distance import geodesic
from geopy.point import Point
import logging
import socket

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app, cors_allowed_origins="*")

# Configuration
NUM_DRONES = 10  # Match Java backend (15001–15010)
BASE_PORT = 15001
HOST = '192.168.0.182'  # Target PC IP address
UPDATE_INTERVAL = 1.0
TRAVEL_TIME = 1000
TRAVEL_DISTANCE = 5000

# Global state for telemetry data
telemetry_data = {}
lock = threading.Lock()

class DroneSimulator:
    def __init__(self, drone_id, port):
        self.drone_id = drone_id
        self.port = port
        self.connection = None
        self.source_lat = 35.0764119 + random.uniform(-0.01, 0.01)
        self.source_lon = 129.0907938 + random.uniform(-0.01, 0.01)
        self.bearing = random.uniform(0, 360)
        source = Point(latitude=self.source_lat, longitude=self.source_lon)
        destination = geodesic(meters=TRAVEL_DISTANCE).destination(source, self.bearing)
        self.dest_lat = destination.latitude
        self.dest_lon = destination.longitude
        self.current_lat = self.source_lat
        self.current_lon = self.source_lon
        self.current_alt = 0.0
        self.ground_speed = TRAVEL_DISTANCE / TRAVEL_TIME  # m/s
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
        self.update_telemetry()

    def _generate_dummy_waypoints(self):
        waypoints = []
        waypoints.append({
            'seq': 0,
            'lat': self.source_lat,
            'lon': self.source_lon,
            'alt': 30.0
        })
        waypoints.append({
            'seq': 1,
            'lat': self.dest_lat,
            'lon': self.dest_lon,
            'alt': 50.0
        })
        return waypoints

    def connect_udp(self):
        for attempt in range(5):
            try:
                # Use udpout to send data to the target IP without waiting for a heartbeat
                self.connection = mavutil.mavlink_connection(
                    f'udpout:{HOST}:{self.port}',
                    source_system=1,
                    source_component=1,
                    dialect='common'
                )
                logger.info(f"Drone {self.drone_id} configured to send to {HOST}:{self.port}")
                return
            except socket.error as e:
                logger.error(f"Attempt {attempt + 1} failed to configure drone {self.drone_id} to {HOST}:{self.port}: {e}")
                self.connection = None
                time.sleep(2)
            finally:
                if self.connection is None and attempt < 4:
                    try:
                        self.connection.close()
                    except:
                        pass
        logger.error(f"Drone {self.drone_id} failed to configure after 5 attempts")

    def send_heartbeat(self):
        if self.connection:
            self.connection.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0
            )

    def send_mission_count(self):
        if self.connection:
            self.connection.mav.mission_count_send(1, 1, self.mission_count)

    def send_mission_item_int(self, waypoint):
        if self.connection:
            self.connection.mav.mission_item_int_send(
                1, 1, waypoint['seq'],
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, 0, 0, 0, 0, waypoint['alt'],
                int(waypoint['lat'] * 1e7), int(waypoint['lon'] * 1e7), 0
            )

    def send_global_position_int(self):
        if self.connection:
            ground_speed_cms = int(min(max(self.ground_speed * 100, -32767), 32767))  # cm/s
            vertical_speed_cms = int(min(max(self.vertical_speed * 100, -32767), 32767))  # cm/s
            heading_cdeg = int(min(max(self.bearing * 100, 0), 35999))  # 0–359.99°
            logger.debug(f"Drone {self.drone_id} GLOBAL_POSITION_INT: lat={self.current_lat*1e7}, lon={self.current_lon*1e7}, alt={self.current_alt*1000}, ground_speed={ground_speed_cms}, vertical_speed={vertical_speed_cms}, heading={heading_cdeg}")
            self.connection.mav.global_position_int_send(
                0, int(self.current_lat * 1e7), int(self.current_lon * 1e7),
                int(self.current_alt * 1000), int(self.current_alt * 1000),
                ground_speed_cms, ground_speed_cms, vertical_speed_cms, heading_cdeg
            )

    def send_nav_controller_output(self):
        if self.connection:
            self.connection.mav.nav_controller_output_send(
                0, 0, 0, int(self.wp_dist), 0, 0, 0, 0, 0
            )

    def send_vfr_hud(self):
        if self.connection:
            throttle_percent = int(min(max(((self.ch3out - 1000) / 1000.0) * 100, 0), 100))  # 0–100
            logger.debug(f"Drone {self.drone_id} VFR_HUD: airspeed={self.airspeed}, ground_speed={self.ground_speed}, heading={self.bearing}, throttle={throttle_percent}, alt={self.current_alt}, climb={self.vertical_speed}")
            self.connection.mav.vfr_hud_send(
                float(self.airspeed), float(self.ground_speed), int(self.bearing),
                throttle_percent, float(self.current_alt), float(self.vertical_speed)
            )

    def send_attitude(self):
        if self.connection:
            self.connection.mav.attitude_send(
                0, float(math.radians(self.roll)), float(math.radians(self.pitch)),
                float(math.radians(self.yaw)), 0, 0, 0
            )

    def send_sys_status(self):
        if self.connection:
            # Includes all 13 arguments for SYS_STATUS
            self.connection.mav.sys_status_send(
                0,  # onboard_control_sensors_present
                0,  # onboard_control_sensors_enabled
                0,  # onboard_control_sensors_health
                0,  # load (not used in simulation)
                int(self.battery_voltage * 1000),  # voltage_battery (mV)
                int(self.battery_current * 100),    # current_battery (cA)
                -1,  # battery_remaining (-1 = unknown)
                0,   # drop_rate_comm (not used)
                0,   # errors_comm (not used)
                0,   # errors_count1
                0,   # errors_count2
                0,   # errors_count3
                0    # errors_count4
            )

    def send_servo_output_raw(self):
        if self.connection:
            # Includes 11 required arguments
            self.connection.mav.servo_output_raw_send(
                0,  # time_usec
                0,  # port
                1000,  # servo1_raw
                1000,  # servo2_raw
                self.ch3out,  # servo3_raw (throttle)
                1000,  # servo4_raw
                1000,  # servo5_raw
                1000,  # servo6_raw
                1000,  # servo7_raw
                1000,  # servo8_raw
                0     # rssi (optional, set to 0)
            )

    def send_wind(self):
        # Commented out due to missing wind_send method
        # if self.connection:
        #     self.connection.mav.wind_send(0, float(self.bearing), float(self.wind_speed), 0.0)
        pass

    def send_gps_raw_int(self):
        if self.connection:
            ground_speed_cms = int(min(max(self.ground_speed * 100, -32767), 32767))  # cm/s
            logger.debug(f"Drone {self.drone_id} GPS_RAW_INT: lat={self.current_lat*1e7}, lon={self.current_lon*1e7}, alt={self.current_alt*1000}, vel={ground_speed_cms}, hdop={self.gps_hdop*100}")
            self.connection.mav.gps_raw_int_send(
                0, 1, int(self.current_lat * 1e7), int(self.current_lon * 1e7),
                int(self.current_alt * 1000), 0, int(self.gps_hdop * 100),
                ground_speed_cms, 0, 8, 0, 0, 0, 0, 0
            )

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
            self.current_alt += self.vertical_speed * UPDATE_INTERVAL
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

        # Full telemetry for WebSocket (UI)
        telemetry = {
            'port': self.port,
            'GCS_IP': HOST,
            'system_id': 1,
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

        # Log the telemetry data being sent for each drone
        logger.info(f"Drone {self.drone_id} telemetry data: lat={telemetry['lat']}, lon={telemetry['lon']}, alt={telemetry['alt']}, "
                    f"heading={telemetry['heading']}, ground_speed={telemetry['ground_speed']}, vertical_speed={telemetry['vertical_speed']}, "
                    f"ch3out={telemetry['ch3out']}, battery_voltage={telemetry['battery_voltage']}, wind_vel={telemetry['wind_vel']}")

        with lock:
            telemetry_data[self.port] = telemetry
        socketio.emit('telemetry_update', telemetry_data)
        logger.debug(f"Drone {self.drone_id} updated: lat={self.current_lat}, lon={self.current_lon}, alt={self.current_alt}")
        return telemetry

    def run(self):
        self.connect_udp()
        if self.connection is not None:
            try:
                self.send_heartbeat()  # Send initial heartbeat
                self.send_mission_count()
                time.sleep(0.1)
                for wp in self.waypoints:
                    self.send_mission_item_int(wp)
                    time.sleep(0.05)
            except Exception as e:
                logger.error(f"Initial send failed for drone {self.drone_id}: {e}")
                self.connection = None

        while True:
            self.update_telemetry()
            if self.connection is None:
                self.connect_udp()
            if self.connection is not None:
                try:
                    self.send_heartbeat()
                    self.send_global_position_int()
                    self.send_nav_controller_output()
                    self.send_vfr_hud()
                    self.send_attitude()
                    self.send_sys_status()
                    self.send_servo_output_raw()
                    # self.send_wind()  # Commented out due to missing wind_send method
                   # self.send_gps_raw_int()
                except Exception as e:
                    logger.error(f"Send failed for drone {self.drone_id}: {e}. Retrying connection.")
                    self.connection = None
            time.sleep(UPDATE_INTERVAL)

def start_simulators():
    threads = []
    for i in range(NUM_DRONES):
        port = BASE_PORT + i
        drone = DroneSimulator(i + 1, port)
        t = threading.Thread(target=drone.run, daemon=True)
        t.start()
        threads.append(t)
        time.sleep(0.5)
    return threads

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/debug')
def debug():
    with lock:
        return jsonify(dict(telemetry_data))

@socketio.on('connect')
def handle_connect():
    logger.info("Client connected to WebSocket")
    with lock:
        socketio.emit('telemetry_update', telemetry_data)

if __name__ == '__main__':
    logger.info(f"Starting simulation for {NUM_DRONES} drones on ports {BASE_PORT} to {BASE_PORT + NUM_DRONES - 1}")
    logger.info(f"Sending data to {HOST}")
    threads = start_simulators()
    logger.info("Flask server starting on http://localhost:5000")
    socketio.run(app, host='0.0.0.0', port=5000, debug=False)