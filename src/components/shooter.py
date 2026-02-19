from magicbot import feedback, will_reset_to
from phoenix6 import controls
from phoenix6.configs import (
    FeedbackConfigs,
    MotionMagicConfigs,
    TalonFXConfiguration,
)
from phoenix6.hardware import TalonFX
from phoenix6.signals import (
    FeedbackSensorSourceValue,
    MotorAlignmentValue,
    NeutralModeValue,
)
from wpimath import units

from lemonlib.smart import SmartProfile


class Shooter:
    """low level component that directly manages the shooter motors and their configuration. controlled by the shooter controller component, but can also be directly controlled for testing purposes"""

    right_motor: TalonFX
    left_motor: TalonFX
    shooter_profile: SmartProfile
    shooter_gear_ratio: float
    shooter_amps: units.amperes
    tuning_enabled: bool

    shooter_velocity = will_reset_to(0.0)
    shooter_voltage = will_reset_to(0.0)
    manual_control = will_reset_to(False)

    use_motion_magic = will_reset_to(False)
    motion_magic_acceleration = 200 # rps/s, TUNE
    motion_magic_jerk = 2000 # rps/s/s, TUNE

    def setup(self):
        self.shooter_motors_config = TalonFXConfiguration()
        self.shooter_motors_config.motor_output.neutral_mode = NeutralModeValue.COAST
        self.shooter_motors_config.feedback = (
            FeedbackConfigs()
            .with_feedback_sensor_source(FeedbackSensorSourceValue.ROTOR_SENSOR)
            .with_sensor_to_mechanism_ratio(self.shooter_gear_ratio)
        )
        self.shooter_motors_config.motion_magic = (
            MotionMagicConfigs()
            .with_motion_magic_acceleration(self.motion_magic_acceleration)  # RPS/s - TUNE THIS
            .with_motion_magic_jerk(self.motion_magic_jerk)  # RPS/s² - TUNE THIS
        )

        self.shooter_motors_config.current_limits.stator_current_limit = (
            self.shooter_amps
        )
        self.shooter_motors_config.current_limits.stator_current_limit_enable = True

        self.left_motor.configurator.apply(self.shooter_motors_config)
        self.right_motor.configurator.apply(self.shooter_motors_config)

        self.shooter_control = (
            controls.VelocityVoltage(0).with_enable_foc(True).with_slot(0)
        )
        self.shooter_motion_magic_control = (
            controls.MotionMagicVelocityVoltage(0).with_enable_foc(True).with_slot(0)
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

    def set_velocity(self, speed: float, use_motion_magic : bool = False):
        self.manual_control = False
        self.use_motion_magic = use_motion_magic
        self.shooter_velocity = speed

    def set_voltage(self, volts: float):
        self.manual_control = True
        self.shooter_voltage = max(0, min(volts, 12))

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
            self.left_motor.set_control(controls.VoltageOut(self.shooter_voltage).with_enable_foc(True))
        elif self.use_motion_magic:
            self.left_motor.set_control(self.shooter_motion_magic_control.with_velocity(self.shooter_velocity))
        else:
            self.left_motor.set_control(self.shooter_control.with_velocity(self.shooter_velocity))
        self.right_motor.set_control(self.shooter_follower)
