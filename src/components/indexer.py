from phoenix6.hardware import TalonFX
from phoenix6.configs.talon_fx_configs import TalonFXConfiguration
from phoenix6.signals import NeutralModeValue
from magicbot import will_reset_to
from wpimath import units

class Inxexer:
    indexMotor: TalonFX
    kickerMotor: TalonFX
    voltage = will_reset_to(0.0)

    def setup(self):
        config = TalonFXConfiguration()
        config.motor_output.neutral_mode = NeutralModeValue.COAST
        self.indexMotor.configurator.apply(config)
        self.kickerMotor.configurator.apply(config)

    def set_voltage(self, voltage: units.volts):
        self.voltage = voltage

    def execute(self):
        self.indexMotor.setVoltage(self.voltage)
        self.kickerMotor.setVoltage(self.voltage)
