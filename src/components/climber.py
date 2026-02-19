from phoenix6.hardware import TalonFX
from phoenix6.configs.talon_fx_configs import TalonFXConfiguration
from phoenix6.signals import NeutralModeValue, MotorAlignmentValue
from phoenix6.controls import Follower
from wpilib import DutyCycleEncoder
from lemonlib.util.alert import  AlertManager, AlertType
from wpimath.units import volts
from magicbot import feedback
class Climber:
    beDown: bool = False
    isLocked: bool = False
    voltsToApply: volts = 0.0
    def setup(self):
        leftConfig = TalonFXConfiguration()
        leftConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.left_motor = TalonFX(-1)#THESE CAN IDS ARE TEMP
        self.left_motor.configurator.apply(leftConfig)

        rightConfig = TalonFXConfiguration()
        rightConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.right_motor = TalonFX(-1)
        self.right_motor.configurator.apply(rightConfig)
        
        #left follows right
        self.left_motor.set_control(
            Follower(self.right_motor.device_id, MotorAlignmentValue.OPPOSED)
        )
        self.encoder = DutyCycleEncoder(2)
    
    def go_down(self):
        self.beDown = True

    def go_up(self):
        self.beDown = False

    @feedback
    def current_volatge(self) -> volts:
        return self.voltsToApply
    def execute(self):
        if self.isLocked:
            AlertManager.instant_alert("[ERROR] Absolute encoder is veryu wrong", AlertType.ERROR)
            return
        #straight up is 0.25
        #0.5 is striaght back, this is not possible and something is very very wrong
        if self.encoder.get() > 0.5:
            self.isLocked = True
            return
        targetPosition = 0.0 if self.beDown else 0.25
        currentPosition = self.encoder.get()
        TOLLERANCE = 0.001
        if abs(currentPosition - targetPosition) < TOLLERANCE:
            #things are great, maybe apply kG if needed
            return
        if currentPosition > targetPosition:
            self.voltsToApply = -10
        else:
            self.voltsToApply = 10

        self.right_motor.setVoltage(self.voltsToApply)