from pymavlink import mavutil
from MAVProxy.modules.lib import mp_module
import serial
import threading
import time
import math

class FollowGCSModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(FollowGCSModule, self).__init__(mpstate, "followgcs", "Follow Ground Station Coordinates with Offset and multi_vehicle", multi_vehicle=True)
        self.add_command("start", self.cmd_start, "Start/Stop following the GSC GPS")
        self.add_command("alt", self.cmd_set_altitude, "Set target altitude")
        self.add_command("radius", self.cmd_set_acceptance_radius, "Set acceptance radius")
        self.add_command("device", self.cmd_set_gps_device, "Set GPS device path")
        self.add_command("baud", self.cmd_set_baud_rate, "Set GPS device baud rate")

        self.altitude = 5.0  # Target altitude (meters)
        self.acceptance_radius = 2.0  # Acceptance radius (meters)
        self.gps_device = "/dev/ttyACM0"  # GPS device path
        self.baud_rate = 9600  # GPS device baud rate

        self.offset_x = 0.0  # Offset in the X direction (meters, relative to GCS)
        self.offset_y = 0.0  # Offset in the Y direction (meters, relative to GCS)

        self.running = False
        self.gps_thread = None
        self.target_coords = None

    def cmd_start(self, args):
        """Command to start/stop following GSC"""
        if len(args) == 0:
            self.running = not self.running
        elif args[0].lower() in ["start", "on"]:
            self.running = True
        elif args[0].lower() in ["stop", "off"]:
            self.running = False
        else:
            self.console.error("Usage: followgcs [start|stop]")
            return

        if self.running:
            self.console.writeln("Follow GSC: Starting")
            if not self.gps_thread or not self.gps_thread.is_alive():
                self.gps_thread = threading.Thread(target=self._gps_loop, daemon=True)
                self.gps_thread.start()
        else:
            self.console.writeln("Follow GSC: Stopping")

    def cmd_set_altitude(self, args):
        """Command to set target altitude."""
        self.console.writeln("Target altitude is fixed at 5 meters and cannot be changed.")

    def cmd_set_acceptance_radius(self, args):
        """Command to set acceptance radius."""
        self.console.writeln("Acceptance radius is fixed at 2 meters and cannot be changed.")

    def cmd_set_gps_device(self, args):
        """Command to set GPS device path."""
        if len(args) != 1:
            self.console.error("Usage: device <device_path>")
            return
        self.gps_device = args[0]
        self.console.writeln(f"GPS device set to {self.gps_device}")

    def cmd_set_baud_rate(self, args):
        """Command to set GPS device baud rate."""
        if len(args) != 1:
            self.console.error("Usage: baud <baud_rate>")
            return
        try:
            self.baud_rate = int(args[0])
            self.console.writeln(f"GPS baud rate set to {self.baud_rate}")
        except ValueError:
            self.console.error("Invalid baud rate value")

    def _gps_loop(self):
        """Thread loop to read GPS data and send follow commands."""
        while self.running:
            try:
                with serial.Serial(self.gps_device, self.baud_rate, timeout=1) as gps_serial:
                    while self.running:
                        line = gps_serial.readline().decode('ascii', errors='ignore').strip()
                        if line.startswith('$GPGGA'):
                            self._process_gps_data(line)
                        # Process joystick input dynamically
                        self._process_joystick_input()
            except serial.SerialException as e:
                self.console.error(f"GPS device error: {e}")
                time.sleep(5)

    def _process_gps_data(self, nmea_sentence):
        """Process NMEA GPGGA sentence and send follow commands."""
        fields = nmea_sentence.split(',')
        if len(fields) < 10 or not fields[2] or not fields[4]:
            self.console.writeln("Invalid GPS data received")
            return

        lat = self._nmea_to_decimal(fields[2], fields[3])
        lon = self._nmea_to_decimal(fields[4], fields[5])

        self.target_coords = (lat, lon)

        # Apply the offset to the target coordinates
        offset_lat, offset_lon = self._apply_offset(lat, lon, self.offset_x, self.offset_y)
        self.console.writeln(f"Target coordinates with offset: {offset_lat}, {offset_lon}")

        # Send MAVLink command to follow target coordinates with offset
        self._send_position_target(offset_lat, offset_lon, self.altitude)

    def _apply_offset(self, lat, lon, offset_x, offset_y):
        """Apply an offset in meters to the latitude and longitude."""
        earth_radius = 6378137.0  # Earth radius in meters

        # Offset in latitude
        delta_lat = offset_y / earth_radius
        offset_lat = lat + (delta_lat * (180.0 / math.pi))

        # Offset in longitude
        delta_lon = offset_x / (earth_radius * math.cos(lat * (math.pi / 180.0)))
        offset_lon = lon + (delta_lon * (180.0 / math.pi))

        return offset_lat, offset_lon

    def _nmea_to_decimal(self, value, direction):
        """Convert NMEA latitude/longitude to decimal degrees."""
        degrees = float(value[:2 if direction in ['N', 'S'] else 3])
        minutes = float(value[2 if direction in ['N', 'S'] else 3:])
        decimal = degrees + (minutes / 60)
        if direction in ['S', 'W']:
            decimal *= -1
        return decimal

    def mavlink_packet(self, m):
        """Handle incoming MAVLink packets for joystick inputs."""
        self.console.writeln(f"Target system: {self.master.target_system}, Packet target: {m.target}")
        self.console.writeln(f"Received packet type: {m.get_type()}")
        if m.get_type() == "MANUAL_CONTROL":
            self.console.writeln(f"Received MANUAL_CONTROL: X={m.x}, Y={m.y}")
            self._update_offsets(m.x, m.y)

    def _process_joystick_input(self):
        """Handle joystick input dynamically."""
        if self.master:
            joystick_data = self.master.recv_match(type='MANUAL_CONTROL', blocking=False)
            if joystick_data:
                self.console.writeln(f"Joystick data received: X={joystick_data.x}, Y={joystick_data.y}, Z={joystick_data.z}, R={joystick_data.r}")
                self._update_offsets(joystick_data.x, joystick_data.y)

    def _update_offsets(self, joystick_x, joystick_y):
        """Update the offset based on joystick input and send updated position."""
        scaling_factor = 0.1  # Adjust sensitivity as needed
        delta_x = joystick_x * scaling_factor
        delta_y = joystick_y * scaling_factor

        self.offset_x += delta_x
        self.offset_y += delta_y

        self.console.writeln(f"Offsets updated: X={self.offset_x}, Y={self.offset_y}")

        # Apply the updated offsets to the current GCS coordinates
        if self.target_coords:
            lat, lon = self.target_coords
            offset_lat, offset_lon = self._apply_offset(lat, lon, self.offset_x, self.offset_y)
            self._send_position_target(offset_lat, offset_lon, self.altitude)

    def _send_position_target(self, lat, lon, altitude):
        """Send MAVLink command to update position target."""
        if self.master:
            self.master.mav.set_position_target_global_int_send(
                0,  # time_boot_ms
                self.settings.target_system,
                self.settings.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                int(0b0000111111111000),  # type_mask
                int(lat * 1e7),  # latitude (scaled to int)
                int(lon * 1e7),  # longitude (scaled to int)
                altitude,  # altitude
                0, 0, 0,  # velocity (ignored)
                0, 0, 0,  # acceleration (ignored)
                0, 0  # yaw, yaw_rate (ignored)
            )

    def unload(self):
        """Cleanup resources when the module is unloaded."""
        self.running = False
        if self.gps_thread and self.gps_thread.is_alive():
            self.gps_thread.join()

def init(mpstate):
    return FollowGCSModule(mpstate)

