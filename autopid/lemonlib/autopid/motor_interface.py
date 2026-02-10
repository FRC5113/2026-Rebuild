"""
Generic motor interface for tuning different motor controllers.
Abstracts TalonFX and SparkMax control methods.
"""

import math
from abc import ABC, abstractmethod
from enum import Enum
from typing import Callable, Optional

import wpilib
from phoenix6 import controls
from phoenix6.configs import Slot0Configs
from phoenix6.hardware import TalonFX, TalonFXS
from phoenix6.signals import GravityTypeValue, StaticFeedforwardSignValue
from rev import PersistMode, ResetMode, SparkMax, SparkMaxConfig
from wpilib.interfaces import MotorController
from wpimath import units
from wpimath.controller import (
    ArmFeedforward,
    ElevatorFeedforward,
    PIDController,
    ProfiledPIDController,
    ProfiledPIDControllerRadians,
    SimpleMotorFeedforwardMeters,
    SimpleMotorFeedforwardRadians,
)
from wpimath.geometry import Rotation2d
from wpimath.trajectory import TrapezoidProfile, TrapezoidProfileRadians

from ..ctre.gains import MechanismType, PhoenixGains, PhoenixWPILibController
from .tuning_data import GravityType


class MotorType(Enum):
    """Supported motor controller types"""

    TALON_FX = "TalonFX"
    TALON_FXS = "TalonFXS"
    SPARK_MAX = "SparkMax"
    GENERIC = "Generic"


class ControlMode(Enum):
    """Motor control modes"""

    VOLTAGE = "Voltage"
    VELOCITY = "Velocity"
    POSITION = "Position"
    PERCENT = "Percent"
    BRAKE = "Brake"


class MotorInterface(ABC):
    """Abstract interface for motor control during tuning"""

    @abstractmethod
    def set_control(self, mode: ControlMode, value: float = 0.0) -> None:
        """
        Set motor control mode and value.

        Args:
            mode: Control mode (voltage, velocity, position, etc.)
            value: Setpoint value (depends on mode)
        """
        pass

    @abstractmethod
    def get_position(self) -> float:
        """Get current position in mechanism units (radians, meters, etc.)"""
        pass

    @abstractmethod
    def get_velocity(self) -> float:
        """Get current velocity in mechanism units/sec"""
        pass

    @abstractmethod
    def apply_gains(
        self,
        kS: float = 0.0,
        kV: float = 0.0,
        kA: float = 0.0,
        kP: float = 0.0,
        kI: float = 0.0,
        kD: float = 0.0,
        kG: float = 0.0,
        slot: int = 0,
    ) -> bool:
        """
        Apply PID gains to motor controller.

        Args:
            kS: Static friction feedforward
            kV: Velocity feedforward
            kA: Acceleration feedforward
            kP: Proportional gain
            kI: Integral gain
            kD: Derivative gain
            kG: Gravity feedforward
            slot: PID slot number

        Returns:
            True if successful
        """
        pass

    @abstractmethod
    def get_motor_id(self) -> int:
        """Get motor device ID"""
        pass

    @abstractmethod
    def get_motor_type(self) -> MotorType:
        """Get motor controller type"""
        pass

    def periodic(self) -> None:
        """Called periodically for any tasks"""
        pass

    def enable_wpilib_control(self, mechanism_type: "MechanismType") -> bool:
        """Enable WPILib control mode (if supported)"""
        return False

    def disable_wpilib_control(self) -> None:
        """Disable WPILib control mode"""
        pass

    def is_wpilib_control_enabled(self) -> bool:
        """Check if WPILib control mode is active"""
        return False


class TalonFXInterface(MotorInterface):
    """TalonFX motor controller interface with WPILib wrapper (always enabled)"""

    def __init__(
        self,
        motor: TalonFX,
        position_getter: Optional[Callable[[], float]] = None,
        mechanism_type: MechanismType = MechanismType.SIMPLE_VELOCITY,
    ):
        """
        Initialize with a TalonFX motor object.

        Args:
            motor: phoenix6.hardware.TalonFX instance
            position_getter: Optional custom position getter (for mechanisms with gearing)
            mechanism_type: Type of mechanism (ARM, ELEVATOR, POSITION, or SIMPLE_VELOCITY) for gravity compensation
        """
        self.motor = motor
        self.position_getter = position_getter
        self._mechanism_type = mechanism_type

        # WPILib controller (always enabled by default)
        self._current_gains = PhoenixGains()
        self._wpilib_controller = PhoenixWPILibController(
            mechanism_type=mechanism_type, gains=self._current_gains
        )

    def enable_wpilib_control(self, mechanism_type: "MechanismType") -> bool:
        """
        Update WPILib control mechanism type.
        Uses Phoenix units (rotations, rotations/sec) directly.
        """
        self._mechanism_type = mechanism_type
        self._wpilib_controller = PhoenixWPILibController(
            mechanism_type=mechanism_type, gains=self._current_gains
        )
        return True

    def disable_wpilib_control(self) -> None:
        """No-op: WPILib control is always enabled"""
        pass

    def is_wpilib_control_enabled(self) -> bool:
        """Check if WPILib control mode is active"""
        return True

    def set_control(self, mode: ControlMode, value: float = 0.0) -> None:
        """
        Set motor control.
        Uses WPILib controller with Phoenix units for position/velocity.
        """
        if mode in [ControlMode.POSITION, ControlMode.VELOCITY]:
            self._apply_wpilib_control(mode, value)
        elif mode == ControlMode.VOLTAGE:
            self.motor.set_control(controls.VoltageOut(value))
        elif mode == ControlMode.PERCENT:
            self.motor.set_control(controls.DutyCycleOut(value))
        elif mode == ControlMode.BRAKE:
            self.motor.set_control(controls.StaticBrake())

    def _apply_wpilib_control(self, mode: ControlMode, value: float) -> None:
        """Apply control using WPILib controller with Phoenix units"""
        # Get current state in Phoenix units (rotations, rotations/sec)
        current_position_rot = self.motor.get_position().value
        current_velocity_rps = self.motor.get_velocity().value

        if mode == ControlMode.POSITION:
            # Convert radians to rotations for WPILib controller
            setpoint_rot = value / math.tau

            # Calculate angle for gravity compensation (ARM only)
            angle = None
            if self._mechanism_type == MechanismType.ARM:
                angle = value  # Already in radians

            # Calculate in Phoenix units
            output_volts = self._wpilib_controller.calculate(
                current_position=current_position_rot,
                setpoint=setpoint_rot,
                current_velocity=current_velocity_rps,
                setpoint_velocity=0.0,
                angle=angle,
            )
            self.motor.set_control(controls.VoltageOut(output_volts))

        elif mode == ControlMode.VELOCITY:
            # Convert radians/sec to rotations/sec
            setpoint_rps = value / math.tau

            angle = None
            if self._mechanism_type == MechanismType.ARM:
                # Use current position for gravity comp
                angle = current_position_rot * math.tau  # Convert to radians

            output_volts = self._wpilib_controller.calculate_velocity(
                current_velocity=current_velocity_rps,
                setpoint_velocity=setpoint_rps,
                setpoint_acceleration=0.0,
                angle=angle,
            )
            self.motor.set_control(controls.VoltageOut(output_volts))

    def get_position(self) -> float:
        """Returns position in radians or custom units"""
        if self.position_getter:
            return self.position_getter()
        rotations = self.motor.get_position().value
        return rotations * math.tau

    def get_velocity(self) -> float:
        """Returns velocity in radians/sec"""
        rps = self.motor.get_velocity().value
        return rps * math.tau

    def apply_gains(
        self,
        kS: float = 0.0,
        kV: float = 0.0,
        kA: float = 0.0,
        kP: float = 0.0,
        kI: float = 0.0,
        kD: float = 0.0,
        kG: float = 0.0,
        gravity_type: Optional[GravityType] = None,
        static_feedforward_sign: Optional[StaticFeedforwardSignValue] = None,
        slot: int = 0,
    ) -> bool:
        """
        Apply gains to WPILib controller only (no Phoenix config application).

        Gains are in Phoenix units:
        - kP: output per rotation error
        - kD: output per rotations/sec error
        - kV: volts per rotations/sec
        - kA: volts per rotations/sec²
        """
        # Update WPILib controller gains
        self._current_gains = PhoenixGains(
            kP=kP, kI=kI, kD=kD, kS=kS, kV=kV, kA=kA, kG=kG
        )
        self._wpilib_controller.update_gains(self._current_gains)
        return True

    def get_motor_id(self) -> int:
        return self.motor.device_id

    def get_motor_type(self) -> MotorType:
        return MotorType.TALON_FX


class TalonFXSInterface(MotorInterface):
    """TalonFXS motor controller interface with WPILib wrapper (always enabled)"""

    def __init__(
        self,
        motor: TalonFXS,
        position_getter: Optional[Callable[[], float]] = None,
        mechanism_type: MechanismType = MechanismType.SIMPLE_VELOCITY,
    ):
        """
        Initialize with a TalonFXS motor object.

        Args:
            motor: phoenix6.hardware.TalonFXS instance
            position_getter: Optional custom position getter (for mechanisms with gearing)
            mechanism_type: Type of mechanism (ARM, ELEVATOR, POSITION, or SIMPLE_VELOCITY) for gravity compensation
        """
        self.motor = motor
        self.position_getter = position_getter
        self._mechanism_type = mechanism_type

        # WPILib controller (always enabled by default)
        self._current_gains = PhoenixGains()
        self._wpilib_controller = PhoenixWPILibController(
            mechanism_type=mechanism_type, gains=self._current_gains
        )

    def enable_wpilib_control(self, mechanism_type: "MechanismType") -> bool:
        """
        Update WPILib control mechanism type.
        Uses Phoenix units (rotations, rotations/sec) directly.
        """
        self._mechanism_type = mechanism_type
        self._wpilib_controller = PhoenixWPILibController(
            mechanism_type=mechanism_type, gains=self._current_gains
        )
        return True

    def disable_wpilib_control(self) -> None:
        """No-op: WPILib control is always enabled"""
        pass

    def is_wpilib_control_enabled(self) -> bool:
        """Check if WPILib control mode is active"""
        return True

    def set_control(self, mode: ControlMode, value: float = 0.0) -> None:
        """Set motor control using WPILib controller"""
        if mode in [ControlMode.POSITION, ControlMode.VELOCITY]:
            self._apply_wpilib_control(mode, value)
        elif mode == ControlMode.VOLTAGE:
            self.motor.set_control(controls.VoltageOut(value))
        elif mode == ControlMode.PERCENT:
            self.motor.set_control(controls.DutyCycleOut(value))
        elif mode == ControlMode.BRAKE:
            self.motor.set_control(controls.StaticBrake())

    def _apply_wpilib_control(self, mode: ControlMode, value: float) -> None:
        """Apply control using WPILib controller with Phoenix units"""
        # Get current state in Phoenix units (rotations, rotations/sec)
        current_position_rot = self.motor.get_position().value
        current_velocity_rps = self.motor.get_velocity().value

        if mode == ControlMode.POSITION:
            # Convert radians to rotations for WPILib controller
            setpoint_rot = value / math.tau

            angle = None
            if self._mechanism_type == MechanismType.ARM:
                angle = value  # Already in radians

            output_volts = self._wpilib_controller.calculate(
                current_position=current_position_rot,
                setpoint=setpoint_rot,
                current_velocity=current_velocity_rps,
                setpoint_velocity=0.0,
                angle=angle,
            )
            self.motor.set_control(controls.VoltageOut(output_volts))

        elif mode == ControlMode.VELOCITY:
            # Convert radians/sec to rotations/sec
            setpoint_rps = value / math.tau

            angle = None
            if self._mechanism_type == MechanismType.ARM:
                angle = current_position_rot * math.tau  # Convert to radians

            output_volts = self._wpilib_controller.calculate_velocity(
                current_velocity=current_velocity_rps,
                setpoint_velocity=setpoint_rps,
                setpoint_acceleration=0.0,
                angle=angle,
            )
            self.motor.set_control(controls.VoltageOut(output_volts))

    def get_position(self) -> float:
        """Returns position in radians or custom units"""
        if self.position_getter:
            return self.position_getter()
        rotations = self.motor.get_position().value
        return rotations * math.tau

    def get_velocity(self) -> float:
        """Returns velocity in radians/sec"""
        rps = self.motor.get_velocity().value
        return rps * math.tau

    def apply_gains(
        self,
        kS: float = 0.0,
        kV: float = 0.0,
        kA: float = 0.0,
        kP: float = 0.0,
        kI: float = 0.0,
        kD: float = 0.0,
        kG: float = 0.0,
        gravity_type: Optional[GravityType] = None,
        static_feedforward_sign: Optional[StaticFeedforwardSignValue] = None,
        slot: int = 0,
    ) -> bool:
        """Apply gains to WPILib controller only (no Phoenix config application)"""
        # Update WPILib controller gains
        self._current_gains = PhoenixGains(
            kP=kP, kI=kI, kD=kD, kS=kS, kV=kV, kA=kA, kG=kG
        )
        self._wpilib_controller.update_gains(self._current_gains)
        return True

    def get_motor_id(self) -> int:
        return self.motor.device_id

    def get_motor_type(self) -> MotorType:
        return MotorType.TALON_FXS


class SparkMaxInterface(MotorInterface):
    """SparkMax motor controller interface"""

    def __init__(
        self,
        motor: SparkMax,
        conversion_factor: float = 1.0,
        position_getter: Optional[Callable[[], float]] = None,
    ):
        """
        Initialize with a SparkMax motor object.

        Args:
            motor: rev.CANSparkMax instance
            conversion_factor: Position conversion factor (e.g., for encoder to mechanism units)
            position_getter: Optional custom position getter
        """
        self.motor = motor
        self.encoder = motor.getEncoder()
        self.pid_controller = motor.getClosedLoopController()
        self.conversion_factor = conversion_factor
        self.position_getter = position_getter
        self._last_config_time = 0.0
        self._config_delay = 0.1
        self.gains = {
            "kS": 0.0,
            "kV": 0.0,
            "kA": 0.0,
            "kP": 0.0,
            "kI": 0.0,
            "kD": 0.0,
            "kG": 0.0,
        }
        self.controller_set = False

    def set_control(self, mode: ControlMode, value: float = 0.0) -> None:
        if mode == ControlMode.VOLTAGE:
            self.motor.setVoltage(value)

        elif mode == ControlMode.VELOCITY:
            if self.controller_set == False:
                self.pid_controller = PIDController(
                    self.gains["kP"],
                    self.gains["kI"],
                    self.gains["kD"],
                )
                self.feedforward = SimpleMotorFeedforwardMeters(
                    self.gains["kS"],
                    self.gains["kV"],
                    self.gains["kA"],
                )
            output = self.feedforward.calculate(value) + self.pid_controller.calculate(
                self.get_velocity(), value
            )
            self.motor.setVoltage(output)

        elif mode == ControlMode.POSITION:
            if self.controller_set == False:
                self.pid_controller = PIDController(
                    self.gains["kP"],
                    self.gains["kI"],
                    self.gains["kD"],
                )
                self.feedforward = SimpleMotorFeedforwardMeters(
                    self.gains["kS"],
                    self.gains["kV"],
                    self.gains["kA"],
                )
            output = self.feedforward.calculate(value) + self.pid_controller.calculate(
                self.get_velocity(), value
            )
            self.motor.setVoltage(output)

        elif mode == ControlMode.PERCENT:
            self.motor.set(value)

        elif mode == ControlMode.BRAKE:
            self.motor.set(0.0)
            self.motor.configure(
                SparkMaxConfig.setIdleMode(SparkMaxConfig.IdleMode.kBrake),
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters,
            )

    def get_position(self) -> float:
        """Returns position in mechanism units (based on conversion factor)"""
        if self.position_getter:
            return self.position_getter()
        return self.encoder.getPosition() * self.conversion_factor

    def get_velocity(self) -> float:
        """Returns velocity in mechanism units/sec (based on conversion factor)"""
        return self.encoder.getVelocity() * self.conversion_factor / 60.0

    def apply_gains(
        self,
        kS: float = 0.0,
        kV: float = 0.0,
        kA: float = 0.0,
        kP: float = 0.0,
        kI: float = 0.0,
        kD: float = 0.0,
        kG: float = 0.0,
        slot: int = 0,
    ) -> bool:
        current_time = wpilib.Timer.getFPGATimestamp()
        if current_time - self._last_config_time < self._config_delay:
            return False

        self.pid_controller.setP(kP)
        self.pid_controller.setI(kI)
        self.pid_controller.setD(kD)
        self.feedforward.setKa(kA)
        self.feedforward.setKv(kV)
        self.feedforward.setKs(kS)

        self._last_config_time = current_time
        return True

    def get_motor_id(self) -> int:
        return self.motor.getDeviceId()

    def get_motor_type(self) -> MotorType:
        return MotorType.SPARK_MAX


class WPIMotorControllerInterface(MotorInterface):
    """Generic WPILib motor controller interface that uses wpilib pidf"""

    def __init__(
        self,
        motor: MotorController,
        position_getter: Callable[[], Rotation2d],
        velocity_getter: Callable[[], units.meters_per_second],
        conversion_factor: float = 1.0,
    ):
        """
        Initialize with a MotorController motor object.

        Args:
            motor: MotorController instance
            position_getter: position getter
            conversion_factor: Position conversion factor (e.g., for encoder to mechanism units)
        """
        self.motor = motor
        self.pid_controller = PIDController(0.0, 0.0, 0.0)
        self.feedforward = SimpleMotorFeedforwardMeters(0.0, 0.0)
        self.conversion_factor = conversion_factor
        self.position_getter = position_getter
        self.velocity_getter = velocity_getter
        self._last_config_time = 0.0
        self._config_delay = 0.1
        self.gains = {
            "kS": 0.0,
            "kV": 0.0,
            "kA": 0.0,
            "kP": 0.0,
            "kI": 0.0,
            "kD": 0.0,
            "kG": 0.0,
        }
        self.controller_set = False

    def set_control(self, mode: ControlMode, value: float = 0.0) -> None:

        if mode == ControlMode.VOLTAGE:
            self.motor.setVoltage(value)

        elif mode == ControlMode.VELOCITY:
            if self.controller_set == False:
                self.pid_controller = PIDController(
                    self.gains["kP"],
                    self.gains["kI"],
                    self.gains["kD"],
                )
                self.feedforward = SimpleMotorFeedforwardMeters(
                    self.gains["kS"],
                    self.gains["kV"],
                    self.gains["kA"],
                )
            output = self.feedforward.calculate(value) + self.pid_controller.calculate(
                self.get_velocity(), value
            )
            self.motor.setVoltage(output)

        elif mode == ControlMode.POSITION:
            if self.controller_set == False:
                self.pid_controller = PIDController(
                    self.gains["kP"],
                    self.gains["kI"],
                    self.gains["kD"],
                )
                self.feedforward = SimpleMotorFeedforwardMeters(
                    self.gains["kS"],
                    self.gains["kV"],
                    self.gains["kA"],
                )
            output = self.feedforward.calculate(value) + self.pid_controller.calculate(
                self.get_velocity(), value
            )
            self.motor.setVoltage(output)

        elif mode == ControlMode.PERCENT:
            self.motor.set(value)

        elif mode == ControlMode.BRAKE:
            self.motor.set(0.0)
            self.motor.stopMotor()

    def get_position(self) -> float:
        """Returns position in Rotation2d"""
        return self.position_getter()

    def get_velocity(self) -> float:
        """Returns velocity in meters/sec"""
        return self.velocity_getter()

    def apply_gains(
        self,
        kS: float = 0.0,
        kV: float = 0.0,
        kA: float = 0.0,
        kP: float = 0.0,
        kI: float = 0.0,
        kD: float = 0.0,
        kG: float = 0.0,
        slot: int = 0,
    ) -> bool:
        current_time = wpilib.Timer.getFPGATimestamp()
        if current_time - self._last_config_time < self._config_delay:
            return False

        self.pid_controller.setP(kP)
        self.pid_controller.setI(kI)
        self.pid_controller.setD(kD)
        self.feedforward.setKa(kA)
        self.feedforward.setKv(kV)
        self.feedforward.setKs(kS)

        self._last_config_time = current_time
        return True

    def get_motor_id(self) -> int:
        return 65

    def get_motor_type(self) -> MotorType:
        return MotorType.GENERIC
