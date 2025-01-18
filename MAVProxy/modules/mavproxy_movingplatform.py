#!/usr/bin/env python3
'''
 Drone to follow moving platform with GPS position from GCS. (Based off "test follow-me options in ArduPilot" by Andrew Tridgell)
 Sami Alakus
 January 2025
'''

import sys, os, time, math
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.mavproxy_map import mp_slipmap
from pymavlink import mavutil
import serial
if mp_util.has_wxpython:
    from MAVProxy.modules.lib.mp_menu import *

class MovingPlatformModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(MovingPlatformModule, self).__init__(mpstate, "movingplatform", "movingplatform module")
        self.add_command('movingplatform', self.cmd_movingplatform, "movingplatform control",
                         ['set (SETTINGS)'])
        self.follow_settings = mp_settings.MPSettings([("radius", float, 100.0),
                                                       ("altitude", float, 50.0),
                                                       ("speed", float, 10.0),
                                                       ("type", str, 'guided'),
                                                       ("vehicle_throttle", float, 0.5),
                                                       ("disable_msg", bool, False)])
        self.add_completion_function('(SETTINGS)', self.follow_settings.completion)
        self.target_pos = None
        self.last_update = 0
        self.circle_dist = 0
        self.gcs_gps_port = "/dev/ttyACM0"
        self.gcs_gps_baudrate = 9600

    def read_gcs_gps(self):
        '''Read GPS coordinates from the GCS module.'''
        try:
            with serial.Serial(self.gcs_gps_port, self.gcs_gps_baudrate, timeout=1) as ser:
                line = ser.readline().decode('ascii', errors='ignore').strip()
                if line.startswith("$GPGGA"):
                    gps_data = line.split(',')
                    latitude = self.nmea_to_decimal(float(gps_data[2]), gps_data[3])
                    longitude = self.nmea_to_decimal(float(gps_data[4]), gps_data[5])
                    return latitude, longitude
        except Exception as e:
            print(f"Error reading GCS GPS: {e}")
        return None, None

    def nmea_to_decimal(self, value, direction):
        '''Convert NMEA latitude/longitude to decimal degrees.'''
        degrees = int(value / 100)
        minutes = value - (degrees * 100)
        decimal = degrees + (minutes / 60)
        if direction in ['S', 'W']:
            decimal = -decimal
        return decimal

    def cmd_movingplatform(self, args):
        '''movingplatform command parser'''
        usage = "usage: movingplatform <set>"
        if len(args) == 0:
            print(usage)
            return
        if args[0] == "set":
            self.follow_settings.command(args[1:])
        else:
            print(usage)

    def update_target(self, time_boot_ms):
        '''update target on map'''
        if not self.mpstate.map:
            # don't draw if no map
            return

        latitude, longitude = self.read_gcs_gps()
        if latitude is None or longitude is None:
            return

        now = time_boot_ms * 1.0e-3
        dt = now - self.last_update
        if dt < 0:
            dt = 0
        self.last_update = now

        self.circle_dist += dt * self.follow_settings.speed

        # assume a circle for now
        circumference = math.pi * self.follow_settings.radius * 2
        rotations = math.fmod(self.circle_dist, circumference) / circumference
        angle = math.pi * 2 * rotations
        self.target_pos = mp_util.gps_newpos(latitude, longitude, math.degrees(angle), self.follow_settings.radius)

        icon = self.mpstate.map.icon('camera-small-red.png')
        (lat, lon) = (self.target_pos[0], self.target_pos[1])
        self.mpstate.map.add_object(mp_slipmap.SlipIcon('movingplatform',
                                                        (lat, lon),
                                                        icon, layer='FollowTest', rotation=0, follow=False))

    def idle_task(self):
        '''update vehicle position'''
        pass

    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''
        if not self.mpstate.map:
            # don't draw if no map
            return

        if m.get_type() != 'GLOBAL_POSITION_INT':
            return
        self.update_target(m.time_boot_ms)

        if self.target_pos is None:
            return

        if self.follow_settings.disable_msg:
            return

        if self.follow_settings.type == 'guided':
            # send normal guided mode packet
            self.master.mav.mission_item_int_send(self.settings.target_system,
                                                  self.settings.target_component,
                                                  0,
                                                  self.module('wp').get_default_frame(),
                                                  mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                                  2, 0, 0, 0, 0, 0,
                                                  int(self.target_pos[0]*1.0e7), int(self.target_pos[1]*1.0e7),
                                                  self.follow_settings.altitude)

        elif self.follow_settings.type == 'yaw':
            # display yaw from vehicle to target
            vehicle = (m.lat*1.0e-7, m.lon*1.0e-7)
            vehicle_yaw = math.degrees(self.master.field('ATTITUDE', 'yaw', 0))
            target_bearing = mp_util.gps_bearing(vehicle[0], vehicle[1], self.target_pos[0], self.target_pos[1])
            # wrap the angle from -180 to 180 thus commanding the vehicle to turn left or right
            # note its in centi-degrees so *100
            relyaw = mp_util.wrap_180(target_bearing - vehicle_yaw) * 100

            self.master.mav.command_long_send(self.settings.target_system,
                                                  self.settings.target_component,
                                                  mavutil.mavlink.MAV_CMD_NAV_SET_YAW_SPEED, 0,
                                                  relyaw,
                                                  self.follow_settings.vehicle_throttle,
                                                  0, 0, 0, 0, 0)

def init(mpstate):
    '''initialise module'''
    return MovingPlatformModule(mpstate)
