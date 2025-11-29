"""
PID Controller Module

Implements a PID (Proportional-Integral-Derivative) controller for
smooth gimbal tracking and drone control.
"""

import time
from typing import Optional


class PIDController:
    """
    PID controller for smooth tracking control.

    Uses proportional, integral, and derivative terms to calculate
    control outputs that minimize error over time.

    Example:
        >>> pid = PIDController(kp=1.0, ki=0.1, kd=0.05, output_limits=(-30, 30))
        >>> error = target_position - current_position
        >>> control_output = pid.update(error, dt=0.033)
    """

    def __init__(
        self,
        kp: float = 1.0,
        ki: float = 0.0,
        kd: float = 0.0,
        output_limits: Optional[tuple[float, float]] = None,
        integral_limits: Optional[tuple[float, float]] = None,
    ):
        """
        Initialize PID controller.

        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            output_limits: Tuple of (min, max) output values
            integral_limits: Tuple of (min, max) integral accumulation (anti-windup)
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits
        self.integral_limits = integral_limits

        # State variables
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = None

    def update(self, error: float, dt: Optional[float] = None) -> float:
        """
        Update PID controller with new error measurement.

        Args:
            error: Current error (setpoint - measured_value)
            dt: Time delta since last update (seconds). If None, uses wall time.

        Returns:
            Control output value
        """
        # Calculate time delta
        current_time = time.time()
        if dt is None:
            if self.last_time is None:
                dt = 0.0
            else:
                dt = current_time - self.last_time
        self.last_time = current_time

        # Avoid division by zero
        if dt <= 0:
            dt = 1e-6

        # Proportional term
        p_term = self.kp * error

        # Integral term (with anti-windup)
        self.integral += error * dt
        if self.integral_limits:
            self.integral = max(
                self.integral_limits[0], min(self.integral_limits[1], self.integral)
            )
        i_term = self.ki * self.integral

        # Derivative term
        derivative = (error - self.last_error) / dt
        d_term = self.kd * derivative

        # Calculate output
        output = p_term + i_term + d_term

        # Apply output limits
        if self.output_limits:
            output = max(self.output_limits[0], min(self.output_limits[1], output))

        # Store state for next iteration
        self.last_error = error

        return output

    def reset(self):
        """Reset PID controller state."""
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = None

    def set_gains(
        self, kp: Optional[float] = None, ki: Optional[float] = None, kd: Optional[float] = None
    ):
        """
        Update PID gains.

        Args:
            kp: Proportional gain (if provided)
            ki: Integral gain (if provided)
            kd: Derivative gain (if provided)
        """
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd

    def get_gains(self) -> dict:
        """
        Get current PID gains.

        Returns:
            Dictionary with kp, ki, kd values
        """
        return {
            "kp": self.kp,
            "ki": self.ki,
            "kd": self.kd,
        }
