import time
import numpy as np


class PID:
    def __init__(self, K_p=0.0, K_i=0.0, K_d=0.0, integral_limit=None):
        """Constructor
        Args:
            K_p (float): The proportional gain
            K_i (float): The integral gain
            K_d (float): The derivative gain
            integral_limit (float, optional): The integral limit
        """
        self.K_p = K_p
        self.K_i = K_i
        self.K_d = K_d
        self.integral_limit = integral_limit

        self.reset()

    def reset(self):
        """Reset the PID controller"""
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()

    def update(self, error, error_derivative=None):
        """Update the PID controller
        Args:
            error (float): The current error
        """
        current_time = time.time()
        dt = current_time - self.last_time

        if dt == 0:
            return 0.0

        self.last_time = current_time

        self.integral = self._get_integral(error, dt)
        if error_derivative is None:
            derivative = self._get_derivative(error, dt)
        else:
            derivative = error_derivative

        output = self.K_p * error + self.K_i * self.integral + self.K_d * derivative

        self.last_error = error

        return output

    def _get_integral(self, error, dt):
        """Calculate the integral term
        Args:
            error (float): The current error
            dt (float): The time delta
        Returns:
            float: The integral term
        """
        integral = self.integral + error * dt

        if self.integral_limit is not None:
            integral = np.clip(integral, -self.integral_limit, self.integral_limit)

        return integral

    def _get_derivative(self, error, dt):
        """Calculate the derivative term
        Args:
            error (float): The current error
            dt (float): The time delta
        Returns:
            float: The derivative term
        """
        derivative = (error - self.last_error) / dt
        return derivative
