from phoenix6.hardware import TalonFX
from phoenix6.configs.talon_fx_configs import TalonFXConfiguration
from phoenix6.signals import NeutralModeValue
from phoenix6.controls import VoltageOut
from phoenix6 import controls
from magicbot import will_reset_to
from wpimath import units


class Intake:
    motor: TalonFX

    voltage = will_reset_to(0.0)

    revolutions = 1000 # 10+ seconds

    def setup(self):
        config = TalonFXConfiguration()
        config.motion_magic.motion_magic_cruise_velocity = 100 # 100 rps
        config.motion_magic.motion_magic_acceleration = 200 # 200 rps/s (0.5 seconds)
        config.motion_magic.motion_magic_jerk = 200 # 200 rps/s/s (0.1 seconds)
        config.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.motor.configurator.apply(config)

    def set_voltage(self, voltage: units.volts):
        self.voltage = voltage

    def intake_forward():
        self.request = controls.MotionMagicVoltage(self.voltage)
        self.motor.set_control(self.request.with_position(revolutions))

    def intake_backward():
        self.request = controls.MotionMagicVoltage(-self.voltage)
        self.motor.set_control(self.request.with_position(revolutions))

    def execute(self):
        pass