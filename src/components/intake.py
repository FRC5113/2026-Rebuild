import enum

from magicbot import feedback, will_reset_to
from phoenix6 import controls
from phoenix6.configs.talon_fx_configs import TalonFXConfiguration
from phoenix6.hardware import TalonFX
from phoenix6.signals import (
    MotorAlignmentValue,
    NeutralModeValue,
)
from wpilib import DutyCycleEncoder
from wpimath import units

from lemonlib.smart import SmartProfile


class IntakeAngle(enum.Enum):
    UP = 5.0
    INTAKING = 85.0


class Intake:
    spin_motor: TalonFX

    left_motor: TalonFX
    right_motor: TalonFX

    left_encoder: DutyCycleEncoder
    right_encoder: DutyCycleEncoder

    profile: SmartProfile

    spin_amps: units.amperes
    arm_amps: units.amperes

    spin_voltage = will_reset_to(0.0)
    arm_voltage = will_reset_to(0.0)
    arm_angle = will_reset_to(IntakeAngle.UP.value)

    def setup(self):
        spin_config = TalonFXConfiguration()
        spin_config.motor_output.neutral_mode = NeutralModeValue.BRAKE
        spin_config.current_limits.supply_current_limit = self.spin_amps
        self.spin_motor.configurator.apply(spin_config)

        arm_config = TalonFXConfiguration()
        arm_config.motor_output.neutral_mode = NeutralModeValue.BRAKE
        arm_config.current_limits.supply_current_limit = self.arm_amps
        self.left_motor.configurator.apply(arm_config)
        self.right_motor.configurator.apply(arm_config)

        self.arm_control = controls.VoltageOut(0).with_enable_foc(True)
        self.arm_follower = controls.Follower(
            self.right_motor.device_id, MotorAlignmentValue.OPPOSED
        )

        self.spin_control = controls.VoltageOut(0).with_enable_foc(True)

    def on_enable(self):
        self.controller = self.profile.create_arm_controller("intake_arm")

    """
    INFORMATIONAL METHODS
    """

    def get_left_angle(self) -> units.degrees:
        return self.left_encoder.get()

    def get_right_angle(self) -> units.degrees:
        return self.right_encoder.get()

    def get_position(self) -> float:
        return (self.get_left_angle() + self.get_right_angle()) / 2

    @feedback
    def get_angle(self) -> units.degrees:
        """Return the angle of the hinge normalized to [-180,180].
        An angle of 0 refers to the intake in the up/stowed position.
        """
        angle = self.get_position() * 360
        if angle > 180:
            angle -= 360
        return angle

    """
    CONTROL METHODS
    """

    def set_voltage(self, voltage: units.volts):
        self.spin_voltage = voltage

    def set_arm_angle(self, angle: units.degrees):
        self.arm_angle = angle

    def execute(self):
        self.arm_voltage = self.controller.calculate(self.get_angle(), self.arm_angle)

        if (self.get_angle() < IntakeAngle.UP.value and self.arm_voltage < 0.0) or (
            self.get_angle() > IntakeAngle.INTAKING.value and self.arm_voltage > 0.0
        ):
            self.arm_voltage = 0.0

        self.right_motor.set_control(self.arm_control.with_output(self.arm_voltage))
        self.left_motor.set_control(self.arm_follower)
        self.spin_motor.set_control(self.spin_control.with_output(self.spin_voltage))
