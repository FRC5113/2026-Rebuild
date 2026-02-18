from magicbot import will_reset_to
from phoenix6.configs.talon_fx_configs import TalonFXConfiguration
from phoenix6 import controls
from phoenix6.hardware import TalonFX
from phoenix6.signals import NeutralModeValue
from wpimath import units


class Intake:
    spin_motor: TalonFX

    left_motor: TalonFX
    right_motor: TalonFX

    voltage = will_reset_to(0.0)

    def setup(self):
        spin_config = TalonFXConfiguration()
        spin_config.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.spin_motor.configurator.apply(spin_config)

        arm_config = TalonFXConfiguration()
        arm_config.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.left_motor.configurator.apply(arm_config)
        self.right_motor.configurator.apply(arm_config)

        self.arm_control = controls.VoltageOut(0)

    def set_voltage(self, voltage: units.volts):
        self.voltage = voltage

    def execute(self):
        motor_control = controls.VoltageOut(self.voltage)
        self.spin_motor.set_control(motor_control)
