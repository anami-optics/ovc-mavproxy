from pymavlink import mavutil
from MAVProxy.modules.lib import mp_module
import serial
import threading
import time

class FollowGCSModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(FollowGCSModule, self).__init__(mpstate, "followgcs", "Follow Ground Station Coordinates")
        self.add_command("followgcs", self.cmd_followgsc, "Start/Stop following the GSC GPS")

        self.altitude = 10.0  # Target altitude (meters)
        self.acceptance_radius = 5.0  # Acceptance radius (meters)
        self.gps_device = "/dev/ttyUSB0"  # GPS device path
        self.baud_rate = 9600  # GPS device baud rate

        self.running = False
        self.gps_thread = None
        self.target_coords = None

    def cmd_followgsc(self, args):
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

    def _gps_loop(self):
        """Thread loop to read GPS data and send follow commands."""
        while self.running:
            try:
                with serial.Serial(self.gps_device, self.baud_rate, timeout=1) as gps_serial:
                    while self.running:
                        line = gps_serial.readline().decode('ascii', errors='ignore').strip()
                        if line.startswith('$GPGGA'):
                            self._process_gps_data(line)
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
        self.console.writeln(f"Target coordinates: {lat}, {lon}")

        # Send MAVLink command to follow target coordinates
        if self.master and self.target_coords:
            self.master.mav.mission_item_send(
                self.settings.target_system,
                self.settings.target_component,
                0,  # sequence
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                2,  # current
                0,  # autocontinue
                0, 0, 0, 0,  # params 1-4 (unused)
                self.target_coords[0],  # latitude
                self.target_coords[1],  # longitude
                self.altitude  # altitude
            )

    def _nmea_to_decimal(self, value, direction):
        """Convert NMEA latitude/longitude to decimal degrees."""
        degrees = float(value[:2 if direction in ['N', 'S'] else 3])
        minutes = float(value[2 if direction in ['N', 'S'] else 3:])
        decimal = degrees + (minutes / 60)
        if direction in ['S', 'W']:
            decimal *= -1
        return decimal

    def unload(self):
        """Cleanup resources when the module is unloaded."""
        self.running = False
        if self.gps_thread and self.gps_thread.is_alive():
            self.gps_thread.join()

    def mavlink_packet(self, m):
        pass

def init(mpstate):
    return FollowGCSModule(mpstate)
