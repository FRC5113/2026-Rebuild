import math

from phoenix6 import controls
from phoenix6.units import ampere
from phoenix6.configs import (
    TalonFXConfiguration,
    CurrentLimitsConfigs,
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
    MotorOutputConfigs,
    Slot0Configs,
    TalonFXConfiguration,
)
from phoenix6.controls import PositionVoltage, VelocityVoltage, VoltageOut
from phoenix6.hardware import CANcoder, Pigeon2, TalonFX
from phoenix6.signals import InvertedValue, NeutralModeValue
from wpimath import units
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState
from wpilib import SmartDashboard

from wpiutil import Sendable, SendableBuilder
from magicbot import will_reset_to
from lemonlib.smart import SmartPreference, SmartProfile
from lemonlib.smart import SmartNT


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
    stopped = will_reset_to(True)
    angle_deadband = SmartPreference(0.0349)

    doing_sysid = will_reset_to(False)

    """
    INITIALIZATION METHODS
    """

    def __init__(self) -> None:
        Sendable.__init__(self)

    def setup(self) -> None:
        """
        This function is automatically called after the motors and encoders have been injected.
        """
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
        self.speed_current_limit_configs = CurrentLimitsConfigs()
        self.direction_current_limit_configs = CurrentLimitsConfigs()

        self.cancoder_config.magnet_sensor.sensor_direction = (
            SensorDirectionValue.CLOCKWISE_POSITIVE
        )

        self.direction_current_limit_configs.stator_current_limit = self.direction_amps
        self.speed_current_limit_configs.stator_current_limit = self.speed_amps

        self.speed_motor_configs.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.direction_motor_configs.motor_output.neutral_mode = NeutralModeValue.BRAKE

        self.direction_motor_configs.feedback = (
            FeedbackConfigs()
            .with_feedback_remote_sensor_id(self.cancoder.device_id)
            .with_feedback_sensor_source(FeedbackSensorSourceValue.FUSED_CANCODER)
            .with_rotor_to_sensor_ratio(self.direction_gear_ratio)
        )

        self.direction_motor_configs.closed_loop_general = (
            ClosedLoopGeneralConfigs().with_continuous_wrap(True)
        )

        self.speed_motor_configs.feedback = (
            FeedbackConfigs().with_sensor_to_mechanism_ratio(
                self.drive_gear_ratio * self.wheel_radius
            )
        )

        self.direction_motor.configurator.apply(self.direction_motor_configs)
        self.direction_motor.configurator.apply(self.direction_current_limit_configs)
        self.speed_motor.configurator.apply(self.speed_motor_configs)
        self.speed_motor.configurator.apply(self.speed_current_limit_configs)

        self.desired_state = None

        self.nt = SmartNT("Swerve Modules")

    def on_enable(self):
        if self.tuning_enabled:
            self.speed_controller = self.speed_profile.create_ctre_flywheel_controller()
            self.direction_controller = (
                self.direction_profile.create_ctre_turret_controller()
            )
            self.direction_motor_configs.slot0 = self.direction_controller

            self.direction_motor.configurator.apply(self.direction_motor_configs)

            self.speed_motor_configs.slot0 = self.speed_controller
            self.speed_motor.configurator.apply(self.speed_motor_configs)

        self.direction_control = (
            controls.PositionVoltage(0).with_slot(0).with_enable_foc(True)
        )
        self.speed_control = (
            controls.VelocityVoltage(0).with_slot(0).with_enable_foc(True)
        )

    """
    INFORMATIONAL METHODS
    """

    def getMeasuredState(self):
        """Retrieve list of measured angle and velocity
        (used for AdvantageScope)
        """

        return [
            self.cancoder.get_absolute_position().value * 360,
            self.speed_motor.get_velocity().value
            * (self.wheel_radius * 2 * math.pi)
            / self.drive_gear_ratio,
        ]

    def getPosition(self) -> SwerveModulePosition:
        return SwerveModulePosition(
            self.speed_motor.get_position().value
            / self.drive_gear_ratio
            * (self.wheel_radius * 2 * math.pi),
            Rotation2d(self.cancoder.get_absolute_position().value * math.tau),
        )

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
            * (self.wheel_radius * 2 * math.pi)
        )

    def get_integrated_angle(self):
        return self.direction_motor.get_position().value * math.tau

    def get_angle_absoulte(self):
        return self.cancoder.get_absolute_position().value * math.tau

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
        """ctre"""
        if self.stopped:
            self.speed_motor.set_control(controls.static_brake.StaticBrake())
            self.direction_motor.set_control(controls.coast_out.CoastOut())
            return
        state = self.desired_state

        current_angle = Rotation2d(self.get_angle_absoulte())

        state.optimize(current_angle)

        target_displacement = state.angle - current_angle
        target_angle = state.angle.radians()

        # m/s to r/s
        state.speed *= self.drive_gear_ratio / (self.wheel_radius * 2 * math.pi)

        # rescale the speed target based on how close we are to being correctly aligned
        target_speed = state.speed * target_displacement.cos()

        # speed_volt = self.drive_ff.calculate(target_speed)
        self.speed_motor.set_control(self.speed_control.with_velocity(target_speed))


        if abs(self.direction_motor.get_closed_loop_error) < 0.03:
            self.direction_motor.set_control(controls.static_brake.StaticBrake())
            return
        
        self.direction_motor.set_control(
            self.direction_control.with_position(target_angle / math.tau)
        )

        # log data to nt
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
            current_angle.radians(),
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