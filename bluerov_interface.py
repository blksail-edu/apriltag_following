from pymavlink import mavutil
import numpy as np


class BlueROV:
    state = None

    def __init__(self, mav_connection):
        self.mav_connection = mav_connection
        self.mav_connection.wait_heartbeat()
        self.mav_connection.set_mode("MANUAL")
        self.mav_connection.arducopter_arm()
        self.mav_connection.motors_armed_wait()
        self.state = "armed"

    def disarm(self):
        """Disarm the ROV, wait for confirmation"""
        self.mav_connection.arducopter_disarm()
        self.mav_connection.motors_disarmed_wait()
        self.state = "disarmed"

    def arm(self):
        """Arm the ROV, wait for confirmation"""
        self.mav_connection.arducopter_arm()
        self.mav_connection.motors_armed_wait()
        self.state = "armed"

    def set_rc_channel(self, channel, pwm):
        """Set a single RC channel"""
        rc_channel_values = [65535 for _ in range(18)]
        rc_channel_values[channel - 1] = pwm
        self.mav_connection.mav.rc_channels_override_send(
            self.mav_connection.target_system,
            self.mav_connection.target_component,
            *rc_channel_values
        )

    def set_rc_channels(self, channels):
        """Set multiple RC channels at once
        Args:
            channels (dict): Dictionary of channel_id: pwm_value
        """
        for channel, value in channels.items():
            pwm_value = 1500 + value * 4
            self.set_rc_channel(channel, pwm_value)

    def set_rc_channels_to_neutral(self):
        """Set all RC channels to neutral (1500)"""
        # set channels 0 to 18 to 1500 (neutral)
        for i in range(0, 18):
            self.set_rc_channel(i, 1500)

    def set_longitudinal_power(self, value):
        """Set the longitudinal power channel"""
        if value > 100 or value < -100:
            print("Longitudinal power must be between -100 and 100")
            print("Cliping value to -100 or 100")
            value = np.clip(value, -100, 100)

        pwm_value = 1500 + value * 4
        self.set_rc_channel(5, pwm_value)

    def set_lateral_power(self, value):
        """Set the lateral power channel"""
        if value > 100 or value < -100:
            print("Lateral power must be between -100 and 100")
            print("Cliping value to -100 or 100")
            value = np.clip(value, -100, 100)

        pwm_value = 1500 + value * 4
        self.set_rc_channel(6, pwm_value)

    def set_vertical_power(self, value):
        """Set the vertical power channel"""
        if value > 100 or value < -100:
            print("Vertical power must be between -100 and 100")
            print("Cliping value to -100 or 100")
            value = np.clip(value, -100, 100)

        pwm_value = 1500 + value * 4
        self.set_rc_channel(4, pwm_value)

    def set_yaw_rate_power(self, value):
        """Set the yaw rate power channel"""
        if value > 100 or value < -100:
            print("Yaw rate power must be between -100 and 100")
            print("Cliping value to -100 or 100")
            value = np.clip(value, -100, 100)

        pwm_value = 1500 + value * 4
        self.set_rc_channel(3, pwm_value)
