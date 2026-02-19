from magicbot import will_reset_to
from phoenix6.configs import TalonFXConfiguration
from phoenix6.controls import VoltageOut
from phoenix6.hardware import TalonFX
from phoenix6.signals import NeutralModeValue
from wpimath import units


class Intake:
    motor: TalonFX

    voltage = will_reset_to(0.0)

    def setup(self):
        config = TalonFXConfiguration()
        config.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.motor.configurator.apply(config)

    def set_voltage(self, voltage: units.volts):
        self.voltage = voltage

    def execute(self):
        motor_control = VoltageOut(self.voltage)
        self.motor.set_control(motor_control)