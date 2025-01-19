from pymavlink import mavutil
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib.mp_settings import MPSetting
import serial
import threading
import time

class FollowGCSJoystickModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(FollowGCSJoystickModule, self).__init__(mpstate, "followgcsjoystick", "Follow Ground Station Coordinates with Joystick")
        self.add_command("followgcsjoystick", self.cmd_followgcsjoystick, "Start/Stop following the GCS GPS with joystick control")

        self.settings = MPSetting(
            [
                ("alt", float, 10.0),
                ("radius", float, 5.0),
                ("device", str, "/dev/ttyUSB0"),
                ("baud", int, 9600)
            ]
        )

        self.add_completion_function(["followgcsjoystick"], self.settings.completion)

        self.running = False
        self.gps_thread = None
        self.target_coords = None
        self.joystick_override = None

    def cmd_followgcsjoystick(self, args):
        """Command to start/stop following GCS"""
        if len(args) == 0:
            self.running = not self.running
        elif args[0].lower() in ["start", "on"]:
            self.running = True
        elif args[0].lower() in ["stop", "off"]:
            self.running = False
        else:
            self.console.error("Usage: followgcsjoystick [start|stop]")
            return

        if self.running:
            self.console.info("Follow GCS Joystick: Starting")
            if not self.gps_thread or not self.gps_thread.is_alive():
                self.gps_thread = threading.Thread(target=self._gps_loop, daemon=True)
                self.gps_thread.start()
        else:
            self.console.info("Follow GCS Joystick: Stopping")

    def _gps_loop(self):
        """Thread loop to read GPS data and send follow commands."""
        while self.running:
            try:
                with serial.Serial(self.settings.device, self.settings.baud, timeout=1) as gps_serial:
                    while self.running:
                        line = gps_serial.readline().decode('ascii', errors='ignore').strip()
                        if line.startswith('$GPGGA'):
                            self._process_gps_data(line)
                        if self.settings.joystick:
                            self._process_joystick_input()
            except serial.SerialException as e:
                self.console.error(f"GPS device error: {e}")
                time.sleep(5)

    def _process_gps_data(self, nmea_sentence):
        """Process NMEA GPGGA sentence and send follow commands."""
        fields = nmea_sentence.split(',')
        if len(fields) < 10 or not fields[2] or not fields[4]:
            self.console.warning("Invalid GPS data received")
            return

        lat = self._nmea_to_decimal(fields[2], fields[3])
        lon = self._nmea_to_decimal(fields[4], fields[5])

        self.target_coords = (lat, lon)
        self.console.info(f"Target coordinates: {lat}, {lon}")

        # Send MAVLink command to follow target coordinates
        if self.master and self.target_coords and not self.joystick_override:
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
                self.settings.alt  # altitude
            )

    def _process_joystick_input(self):
        """Process joystick inputs for manual override."""
        joystick_data = self.get_joystick()
        if joystick_data:
            self.joystick_override = True
            roll, pitch, throttle, yaw = joystick_data
            self.master.mav.manual_control_send(
                self.settings.target_system,
                int(roll * 1000),  # x-axis
                int(pitch * 1000),  # y-axis
                int(throttle * 1000),  # z-axis
                int(yaw * 1000),  # r-axis
                0  # buttons (unused)
            )
        else:
            self.joystick_override = False

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
    return FollowGCSJoystickModule(mpstate)
