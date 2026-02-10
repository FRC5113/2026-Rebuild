import math
from typing import List

from phoenix6 import controls, StatusSignal, BaseStatusSignal
from phoenix6.configs import (
    TalonFXConfiguration,
    CANcoderConfiguration,
)
from phoenix6.hardware import CANcoder, TalonFX
from phoenix6.signals import (
    NeutralModeValue,
    SensorDirectionValue,
    FeedbackSensorSourceValue,
)
from phoenix6.configs import (
    ClosedLoopGeneralConfigs,
    FeedbackConfigs,
    TalonFXConfiguration,
)
from phoenix6.hardware import CANcoder, TalonFX
from phoenix6.signals import NeutralModeValue


from wpimath import units
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState

from wpiutil import Sendable
from magicbot import will_reset_to
from lemonlib.smart import SmartPreference, SmartProfile, SmartNT
from lemonlib.ctre import tryUntilOk


class SwerveWheel(Sendable):
    drive_gear_ratio: float
    direction_gear_ratio: float
    wheel_radius: units.meters
    speed_motor: TalonFX
    speed_profile: SmartProfile
    direction_motor: TalonFX
    direction_profile: SmartProfile
    cancoder: CANcoder

    direction_amps: units.amperes
    speed_amps: units.amperes

    tuning_enabled: bool

    """Module must be explicitly told to move (via setDesiredState) each
    loop, otherwise it defaults to stopped for safety.
    """
    stopped = will_reset_to(True)  # Resets to True each loop if not explicitly set
    angle_deadband = SmartPreference(0.0349)  # ~2 degrees in radians

    doing_sysid = will_reset_to(False)

    signals: List[StatusSignal] = []

    """
    INITIALIZATION METHODS
    """

    def __init__(self) -> None:
        Sendable.__init__(self)

    def setup(self) -> None:
        """
        This function is automatically called after the motors and encoders have been injected.
        """
        # set fault update frequencies
        self.direction_motor.get_fault_field().set_update_frequency(
            frequency_hz=4, timeout_seconds=0.01
        )
        self.speed_motor.get_fault_field().set_update_frequency(
            frequency_hz=4, timeout_seconds=0.01
        )

        # apply configs
        self.speed_motor_configs = TalonFXConfiguration()
        self.direction_motor_configs = TalonFXConfiguration()
        self.cancoder_config = CANcoderConfiguration()

        self.cancoder_config.magnet_sensor.sensor_direction = (
            SensorDirectionValue.CLOCKWISE_POSITIVE
        )

        # current limits
        self.direction_motor_configs.current_limits.stator_current_limit = (
            self.direction_amps
        )
        self.speed_motor_configs.current_limits.stator_current_limit = self.speed_amps

        # neutral modes
        self.speed_motor_configs.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.direction_motor_configs.motor_output.neutral_mode = NeutralModeValue.BRAKE

        # feedback configs - uses fused CANcoder for direction motor
        self.direction_motor_configs.feedback = (
            FeedbackConfigs()
            .with_feedback_remote_sensor_id(self.cancoder.device_id)
            .with_feedback_sensor_source(
                FeedbackSensorSourceValue.FUSED_CANCODER
            )  # Fuse CANcoder with internal encoder for better accuracy
            .with_rotor_to_sensor_ratio(
                self.direction_gear_ratio
            )  # Converts motor rotations to sensor rotations
        )
        # Speed motor feedback converts motor rotations to linear distance (meters)
        self.speed_motor_configs.feedback = (
            FeedbackConfigs().with_sensor_to_mechanism_ratio(
                self.drive_gear_ratio * self.wheel_radius
            )
        )

        # Enable continuous wrap so direction motor can take shortest path to target angle
        self.direction_motor_configs.closed_loop_general = (
            ClosedLoopGeneralConfigs().with_continuous_wrap(True)
        )
        tryUntilOk(
            5,
            lambda: self.direction_motor.configurator.apply(
                self.direction_motor_configs
            ),
        )

        tryUntilOk(
            5, lambda: self.speed_motor.configurator.apply(self.speed_motor_configs)
        )

        self.drive_position = self.speed_motor.get_position()
        self.drive_velocity = self.speed_motor.get_velocity()
        self.direction_position = self.direction_motor.get_position()
        self.direction_velocity = self.direction_motor.get_velocity()

        self.signals = [
            self.drive_position,
            self.drive_velocity,
            self.direction_position,
            self.direction_velocity,
        ]

        self.meters_per_wheel_rotation = self.wheel_radius * math.tau
        self.drive_rot_per_meter = (
            self.drive_gear_ratio / self.meters_per_wheel_rotation
        )

        self.swerve_module_position = SwerveModulePosition()

        self.desired_state = None

        self.nt = SmartNT("Swerve Modules")

    def on_enable(self):
        if self.tuning_enabled:
            self.speed_controller = self.speed_profile.create_ctre_flywheel_controller()
            self.direction_controller = (
                self.direction_profile.create_ctre_turret_controller()
            )
            self.direction_motor_configs.slot0 = self.direction_controller
            self.speed_motor_configs.slot0 = self.speed_controller

            tryUntilOk(
                5,
                lambda: self.direction_motor.configurator.apply(
                    self.direction_motor_configs
                ),
            )

            tryUntilOk(
                5, lambda: self.speed_motor.configurator.apply(self.speed_motor_configs)
            )

        self.direction_control = controls.PositionVoltage(0)

        self.speed_control = controls.VelocityTorqueCurrentFOC(
            0
        )  # FOC = Field Oriented Control for better motion

    """
    INFORMATIONAL METHODS
    """

    def getMeasuredState(self):
        """Retrieve list of measured angle and velocity
        (used for AdvantageScope)
        """

        return [
            self.cancoder.get_absolute_position().value
            * 360,  # Convert rotations to degrees
            self.speed_motor.get_velocity().value
            * (
                self.wheel_radius * 2 * math.pi
            )  # Convert rotations/s to m/s using wheel circumference
            / self.drive_gear_ratio,
        ]

    def getPosition(self, refresh: bool = False) -> SwerveModulePosition:
        if refresh:
            self.drive_position.refresh()
            self.direction_position.refresh()
            self.drive_velocity.refresh()
            self.direction_velocity.refresh()

        drive_rot = BaseStatusSignal.get_latency_compensated_value(
            self.drive_position, self.drive_velocity
        )
        angle_rot = BaseStatusSignal.get_latency_compensated_value(
            self.direction_position, self.direction_velocity
        )

        self.swerve_module_position.distance = (
            drive_rot / self.drive_rot_per_meter
        )  # Convert motor rotations to linear distance (meters)
        self.swerve_module_position.angle = Rotation2d.fromRotations(angle_rot)

        return self.swerve_module_position

    def getVoltage(self) -> units.volts:
        return (
            self.speed_motor.get_motor_voltage().value
            / self.drive_gear_ratio
            * (self.wheel_radius * 2 * math.pi)
        )

    def getVelocity(self):
        return (
            self.speed_motor.get_velocity().value
            / self.drive_gear_ratio
            * (self.wheel_radius * 2 * math.pi)  # Convert rotations/s to m/s
        )

    def get_angle_absoulte(self) -> Rotation2d:
        return Rotation2d(
            self.cancoder.get_absolute_position().value * math.tau
        )  # Convert rotations to radians

    def getSignals(self):
        return self.signals

    def putTelem(self):
        self.nt.put(
            f"Direction {self.direction_motor.device_id}/Error",
            self.direction_motor.get_closed_loop_error().value,
        )
        self.nt.put(
            f"Direction {self.direction_motor.device_id}/Reference",
            self.direction_motor.get_closed_loop_reference().value,
        )
        self.nt.put(
            f"Direction {self.direction_motor.device_id}/Output",
            self.direction_motor.get_closed_loop_output().value,
        )
        self.nt.put(
            f"Direction {self.direction_motor.device_id}/Mesurement",
            self.swerve_module_position.angle.radians(),
        )

        self.nt.put(
            f"Speed {self.speed_motor.device_id}/Error",
            self.speed_motor.get_closed_loop_error().value,
        )
        self.nt.put(
            f"Speed {self.speed_motor.device_id}/Reference",
            self.speed_motor.get_closed_loop_reference().value,
        )
        self.nt.put(
            f"Speed {self.speed_motor.device_id}/Output",
            self.speed_motor.get_closed_loop_output().value,
        )
        self.nt.put(
            f"Speed {self.speed_motor.device_id}/Mesurement",
            self.speed_motor.get_position().value,
        )

    """
    CONTROL METHODS
    """

    def setDesiredState(self, state: SwerveModuleState):
        self.stopped = False
        self.desired_state = state

    def setVoltageOnly(self, voltage: float):
        self.stopped = False
        self.doing_sysid = True
        self.sysid_volts = voltage

    """
    EXECUTE
    """

    def execute(self) -> None:
        if self.stopped:
            self.speed_motor.set_control(controls.static_brake.StaticBrake())
            self.direction_motor.set_control(controls.coast_out.CoastOut())
            return

        # Update position tracking before using it
        self.getPosition(refresh=True)

        state = self.desired_state

        current_angle = self.swerve_module_position.angle

        # Optimize flips the wheel direction if it's faster than rotating 180 degrees
        state.optimize(current_angle)

        target_displacement = state.angle - current_angle
        target_angle = state.angle.radians()

        # Convert m/s to rotations/s for motor control
        state.speed *= self.drive_rot_per_meter

        # Cosine compensation: reduce speed when wheel isn't pointing the right direction
        # This prevents the robot from drifting while the wheel is still rotating
        target_speed = state.speed * target_displacement.cos()

        self.speed_motor.set_control(self.speed_control.with_velocity(target_speed))

        if abs(self.direction_motor.get_closed_loop_error().value) < 0.03:
            self.direction_motor.set_control(controls.static_brake.StaticBrake())
            return

        # Divide by tau to convert radians back to rotations for motor control
        self.direction_motor.set_control(
            self.direction_control.with_position(target_angle / math.tau)
        )
