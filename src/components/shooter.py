import math

from phoenix6 import controls, StatusSignal
from phoenix6.configs import (
    ClosedLoopGeneralConfigs,
    FeedbackConfigs,
    TalonFXConfiguration,
    TalonFXSConfiguration,
    CANcoderConfiguration,
)
from phoenix6.hardware import CANcoder, TalonFX, TalonFXS
from phoenix6.signals import (
    FeedbackSensorSourceValue,
    NeutralModeValue,
    SensorDirectionValue,
    MotorAlignmentValue,
    MotorArrangementValue,
    ExternalFeedbackSensorSourceValue,
)
from wpimath import units
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState
from wpiutil import Sendable

from magicbot import will_reset_to, feedback
from lemonlib.smart import SmartNT, SmartPreference, SmartProfile


class Shooter:
    right_motor: TalonFX
    left_motor: TalonFX
    shooter_profile: SmartProfile
    shooter_gear_ratio: float
    shooter_amps: units.amperes
    tuning_enabled: bool

    shooter_velocity = will_reset_to(0.0)
    shooter_voltage = will_reset_to(0.0)
    manual_control = will_reset_to(False)

    def setup(self):
        self.shooter_motors_config = TalonFXConfiguration()
        self.shooter_motors_config.motor_output.neutral_mode = NeutralModeValue.COAST
        self.shooter_motors_config.feedback = (
            FeedbackConfigs()
            .with_feedback_sensor_source(FeedbackSensorSourceValue.ROTOR_SENSOR)
            .with_sensor_to_mechanism_ratio(self.shooter_gear_ratio)
        )
        self.shooter_motors_config.current_limits.stator_current_limit = (
            self.shooter_amps
        )

        self.left_motor.configurator.apply(self.shooter_motors_config)
        self.right_motor.configurator.apply(self.shooter_motors_config)

        self.shooter_control = (
            controls.VelocityVoltage(0).with_enable_foc(True).with_slot(0)
        )
        self.shooter_follower = controls.Follower(
            self.left_motor.device_id, MotorAlignmentValue.OPPOSED
        )

    def on_enable(self):
        if self.tuning_enabled:
            self.shooter_controller = (
                self.shooter_profile.create_ctre_flywheel_controller()
            )
            self.left_motor.configurator.apply(
                self.shooter_motors_config.with_slot0(self.shooter_controller)
            )
            self.right_motor.configurator.apply(
                self.shooter_motors_config.with_slot0(self.shooter_controller)
            )

    """
    CONTROL METHODS
    """

    def set_velocity(self, speed: float):
        self.manual_control = False
        self.shooter_velocity = speed

    def set_voltage(self, volts: float):
        self.manual_control = True
        self.shooter_voltage = volts

    """
    INFORMATIONAL METHODS
    """

    @feedback
    def get_velocity(self) -> float:
        return self.left_motor.get_velocity().value

    @feedback
    def get_target_velocity(self) -> float:
        return self.shooter_velocity

    def execute(self):
        if self.manual_control:
            self.left_motor.set_control(controls.VoltageOut(self.shooter_voltage))
        else:
            self.left_motor.set_control(
                self.shooter_control.with_velocity(self.shooter_velocity)
            )
        self.right_motor.set_control(self.shooter_follower)
