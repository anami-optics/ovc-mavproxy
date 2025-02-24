from pymavlink import mavutil
from MAVProxy.modules.lib import mp_module
import serial
import threading
import time
import math

class DevModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(DevModule, self).__init__(mpstate, "dev", "Development module for testing", multi_vehicle=True)
        self.add_command("dev start", self.cmd_start, "Start dev module")
        self.add_command("dev stop", self.cmd_start, "Stop dev module")

        self.offset_x = 0.0  # Offset in the X direction (meters, relative to GCS)
        self.offset_y = 0.0  # Offset in the Y direction (meters, relative to GCS)

        self.running = False
        self.positioning_thread = None
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
            self.console.error("Usage: dev [start|stop]")
            return

        if self.running:
            self.console.writeln("Dev: Starting")
            if not self.positioning_thread or not self.positioning_thread.is_alive():
                self.positioning_thread = threading.Thread(target=self._positioning_loop, daemon=True)
                self.positioning_thread.start()
        else:
            self.console.writeln("Dev: Stopping")

    def _positioning_loop(self):
        """Thread loop to read GPS data and send follow commands."""
        while self.running:
            try:
                with serial.Serial(self.gps_device, self.baud_rate, timeout=1) as gps_serial:
                    while self.running:
                        # Process joystick input dynamically
                        self._process_joystick_input()
            except serial.SerialException as e:
                self.console.error(f"GPS device error: {e}")
                time.sleep(5)

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

    def unload(self):
        """Cleanup resources when the module is unloaded."""
        self.running = False
        if self.positioning_thread and self.positioning_thread.is_alive():
            self.positioning_thread.join()

def init(mpstate):
    return DevModule(mpstate)

