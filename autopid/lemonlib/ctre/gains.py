"""
WPILib Controller Wrapper for Phoenix 6 Units
Uses Phoenix 6 units directly (rotations, rotations/sec) with WPILib controllers.
No unit conversion needed - tune with Phoenix values, use with WPILib.
"""

import math
from dataclasses import dataclass
from enum import Enum
from typing import Optional

from wpimath.controller import (
    ArmFeedforward,
    ElevatorFeedforward,
    PIDController,
    SimpleMotorFeedforwardMeters,
)


class MechanismType(Enum):
    """Types of mechanisms with different feedforward models"""

    SIMPLE_VELOCITY = "simple"
    ARM = "arm"
    ELEVATOR = "elevator"
    POSITION = "position"


@dataclass
class PhoenixGains:
    """
    Gains in Phoenix 6 units (rotations, rotations/sec).
    These are the exact values used in Phoenix slot configs.

    PID Gains (dimensionless):
    - kP: Proportional gain (output per rotation error)
    - kI: Integral gain (output per rotation*sec integrated error)
    - kD: Derivative gain (output per rotations/sec error)

    Feedforward Gains (volts):
    - kS: Static friction/voltage offset (volts)
    - kV: Velocity feedforward (volts per rotation/sec)
    - kA: Acceleration feedforward (volts per rotation/sec²)
    - kG: Gravity feedforward (volts)

    All units match Phoenix 6 exactly - no conversion needed.
    """

    kP: float = 0.0
    kI: float = 0.0
    kD: float = 0.0
    kS: float = 0.0
    kV: float = 0.0
    kA: float = 0.0
    kG: float = 0.0


class PhoenixWPILibController:
    """
    WPILib controller that uses Phoenix 6 units directly.

    Input/output units:
    - Position: rotations
    - Velocity: rotations/sec
    - Acceleration: rotations/sec²
    - Output: volts

    This matches Phoenix 6 exactly, so gains tuned here can be
    directly copied to Phoenix slot configs with no conversion.
    """

    def __init__(
        self, mechanism_type: MechanismType, gains: PhoenixGains, period: float = 0.02
    ):
        """
        Args:
            mechanism_type: Type of mechanism (determines feedforward model)
            gains: Initial gains in Phoenix units (rotations, rotations/sec)
            period: Control loop period in seconds
        """
        self.mechanism_type = mechanism_type
        self.gains = gains
        self.period = period

        # PID controller (dimensionless gains work the same)
        self.pid = PIDController(gains.kP, gains.kI, gains.kD, period)

        # Feedforward controller (need to convert internally for WPILib)
        self.feedforward = self._create_feedforward()

    def _create_feedforward(self):
        """
        Create feedforward controller.
        WPILib expects radians/meters, so we convert Phoenix units internally.
        """
        # Convert Phoenix gains (per rotation/sec) to WPILib gains (per rad/sec or m/s)
        # kV_phoenix [V/(rot/s)] = kV_wpilib [V/(rad/s)] * (2π rad/rot)
        # So: kV_wpilib = kV_phoenix / (2π)
        kV_wpilib = self.gains.kV / (2 * math.pi)
        kA_wpilib = self.gains.kA / (2 * math.pi)

        if self.mechanism_type == MechanismType.SIMPLE_VELOCITY:
            return SimpleMotorFeedforwardMeters(self.gains.kS, kV_wpilib)

        elif self.mechanism_type == MechanismType.ARM:
            return ArmFeedforward(self.gains.kS, self.gains.kG, kV_wpilib, kA_wpilib)

        elif self.mechanism_type == MechanismType.ELEVATOR:
            return ElevatorFeedforward(
                self.gains.kS, self.gains.kG, kV_wpilib, kA_wpilib
            )

        elif self.mechanism_type == MechanismType.POSITION:
            return SimpleMotorFeedforwardMeters(self.gains.kS, kV_wpilib, kA_wpilib)

        else:
            raise ValueError(f"Unknown mechanism type: {self.mechanism_type}")

    def calculate(
        self,
        current_position: float,
        setpoint: float,
        current_velocity: Optional[float] = None,
        setpoint_velocity: Optional[float] = None,
        setpoint_acceleration: float = 0.0,
        angle: Optional[float] = None,
    ) -> float:
        """
        Calculate control output for position control.

        Args:
            current_position: Current position (rotations)
            setpoint: Desired position (rotations)
            current_velocity: Current velocity (rotations/sec) - optional
            setpoint_velocity: Desired velocity (rotations/sec)
            setpoint_acceleration: Desired acceleration (rotations/sec²)
            angle: Current angle for arm gravity compensation (radians)

        Returns:
            Control output in volts
        """
        # PID output (dimensionless gains work with any units)
        pid_output = self.pid.calculate(current_position, setpoint)

        # Feedforward output (convert to rad/s for WPILib)
        ff_output = 0.0

        if self.mechanism_type == MechanismType.SIMPLE_VELOCITY:
            if setpoint_velocity is not None:
                # Convert rot/s to rad/s
                vel_rad_s = setpoint_velocity * 2 * math.pi
                ff_output = self.feedforward.calculate(vel_rad_s)

        elif self.mechanism_type == MechanismType.ARM:
            if angle is None:
                raise ValueError("Must provide angle for ARM mechanism")
            if setpoint_velocity is not None:
                vel_rad_s = setpoint_velocity * 2 * math.pi
                accel_rad_s2 = setpoint_acceleration * 2 * math.pi
                ff_output = self.feedforward.calculate(angle, vel_rad_s, accel_rad_s2)
            else:
                ff_output = self.feedforward.calculate(angle, 0.0)

        elif self.mechanism_type == MechanismType.ELEVATOR:
            if setpoint_velocity is not None:
                vel_rad_s = setpoint_velocity * 2 * math.pi
                accel_rad_s2 = setpoint_acceleration * 2 * math.pi
                ff_output = self.feedforward.calculate(vel_rad_s, accel_rad_s2)
            else:
                ff_output = self.feedforward.calculate(0.0)

        elif self.mechanism_type == MechanismType.POSITION:
            if setpoint_velocity is not None:
                vel_rad_s = setpoint_velocity * 2 * math.pi
                accel_rad_s2 = setpoint_acceleration * 2 * math.pi
                ff_output = self.feedforward.calculate(vel_rad_s, accel_rad_s2)

        return pid_output + ff_output

    def calculate_velocity(
        self,
        current_velocity: float,
        setpoint_velocity: float,
        setpoint_acceleration: float = 0.0,
        angle: Optional[float] = None,
    ) -> float:
        """
        Calculate control output for velocity control.

        Args:
            current_velocity: Current velocity (rotations/sec)
            setpoint_velocity: Desired velocity (rotations/sec)
            setpoint_acceleration: Desired acceleration (rotations/sec²)
            angle: Current angle for arm gravity compensation (radians)

        Returns:
            Control output in volts
        """
        # PID on velocity error
        pid_output = self.pid.calculate(current_velocity, setpoint_velocity)

        # Feedforward (convert to rad/s)
        vel_rad_s = setpoint_velocity * 2 * math.pi
        accel_rad_s2 = setpoint_acceleration * 2 * math.pi

        ff_output = 0.0
        if self.mechanism_type == MechanismType.ARM and angle is not None:
            ff_output = self.feedforward.calculate(angle, vel_rad_s, accel_rad_s2)
        elif self.mechanism_type == MechanismType.ELEVATOR:
            ff_output = self.feedforward.calculate(vel_rad_s, accel_rad_s2)
        else:
            ff_output = self.feedforward.calculate(vel_rad_s, accel_rad_s2)

        return pid_output + ff_output

    def update_gains(self, new_gains: PhoenixGains):
        """
        Update controller gains.

        Args:
            new_gains: New gains in Phoenix units (rotations, rotations/sec)
        """
        self.gains = new_gains
        self.pid.setPID(new_gains.kP, new_gains.kI, new_gains.kD)
        self.feedforward = self._create_feedforward()

    def reset(self):
        """Reset PID controller (clear integral accumulator, etc.)"""
        self.pid.reset()

    def at_setpoint(self, tolerance: float = 0.05) -> bool:
        """
        Check if at setpoint.

        Args:
            tolerance: Position tolerance (rotations)

        Returns:
            True if within tolerance
        """
        return abs(self.pid.getPositionError()) < tolerance
