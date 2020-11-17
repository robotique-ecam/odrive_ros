#!/usr/bin/env python3


"""Odrive simulated interface for Webots."""


from math import floor


class ODriveInterfaceSimulator(object):

    def __init__(self, logger):
        self.logger = logger
        self.encoder_cpr = 4096
        self.connected = False
        self.engaged = False

        self.right_axis_vel = 0  # units: encoder counts/s
        self.left_axis_vel = 0
        self.right_axis_pos = 0  # go from 0 up to encoder_cpr-1
        self.left_axis_pos = 0
        self.last_time_update = None

    def update_time(self, curr_time):
        # provided so simulator can update position
        if self.last_time_update is None:
            self.last_time_update = curr_time
            return

        dt = curr_time - self.last_time_update
        self.left_axis_pos = floor(self.left_axis_pos + self.left_axis_vel * dt) % self.encoder_cpr
        self.right_axis_pos = floor(self.right_axis_pos + self.right_axis_vel * dt) % self.encoder_cpr
        self.last_time_update = curr_time

    def connect(self, port=None, right_axis=0, timeout=30):
        if self.connected:
            self.logger.info('Already connected. Simulating disc/reconnect.')

        self.encoder_cpr = 4096
        self.logger.info('Connected to simulated ODrive.')
        return True

    def disconnect(self):
        self.connected = False
        return True

    def calibrate(self):
        if not self.connected:
            self.logger.error('Not connected.')
            return False
        self.logger.info('Calibrated.')
        return True

    def preroll(self, wait=True, reverse=False):
        if not self.connected:
            self.logger.error('Not connected.')
            return False
        return True

    def ensure_prerolled(self):
        return True

    def engaged(self):
        return self.engaged

    def engage(self):
        if not self.connected:
            self.logger.error('Not connected.')
            return False
        self.engaged = True
        return True

    def release(self):
        if not self.connected:
            self.logger.error('Not connected.')
            return False
        self.engaged = False
        return True

    def drive(self, left_motor_val, right_motor_val):
        if not self.connnected:
            self.logger.error('Not connected.')
            return
        self.left_axis.controller.vel_setpoint = left_motor_val
        self.right_axis.controller.vel_setpoint = -right_motor_val

        return True

    def get_errors(self, clear=True):
        if not self.driver:
            return None
        return 'Simulated ODrive, no errors.'

    def left_vel_estimate(self):
        return self.left_axis_vel

    def right_vel_estimate(self):
        return self.right_axis_vel

    def left_pos(self):
        return self.left_axis_pos

    def right_pos(self):
        return self.right_axis_pos

    def left_current(self):
        return 0

    def right_current(self):
        return 0
