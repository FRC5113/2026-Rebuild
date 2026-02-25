import typing

import phoenix6
from phoenix6.configs import FeedbackConfigs, MotorOutputConfigs, TalonFXConfiguration
from phoenix6.controls import Follower, VoltageOut
from phoenix6.signals import MotorAlignmentValue, NeutralModeValue
from wpilib import sysid
from wpimath.units import volts

from subsystems.sysid_subsystem import SysidSubsystem

FollowerDescriptor = tuple[phoenix6.hardware.TalonFX, bool]


class Flywheel(SysidSubsystem):
    def __init__(
        self,
        flywheel_motor: phoenix6.hardware.TalonFX,
        *followers: FollowerDescriptor,
        gearing: float,
        name: str | None = None,
    ) -> None:
        super().__init__()
        self.flywheel = flywheel_motor
        self.followers = [motor for motor, _ in followers]
        if name is not None:
            self.setName(name)

        flywheel_motor_config = MotorOutputConfigs()
        flywheel_motor_config.neutral_mode = NeutralModeValue.COAST
        feedback_config = FeedbackConfigs().with_sensor_to_mechanism_ratio(gearing)

        flywheel_config = self.flywheel.configurator
        flywheel_config.apply(
            TalonFXConfiguration()
            .with_feedback(feedback_config)
            .with_motor_output(flywheel_motor_config)
        )

        for motor, oppose_leader in followers:
            motor.configurator.apply(
                TalonFXConfiguration()
                .with_motor_output(flywheel_motor_config)
                .with_feedback(feedback_config)
            )
            motor.set_control(
                Follower(
                    self.flywheel.device_id,
                    (
                        MotorAlignmentValue.OPPOSED
                        if oppose_leader
                        else MotorAlignmentValue.ALIGNED
                    ),
                )
            )

    # Tell SysId how to plumb the driving voltage to the motors.
    @typing.override
    def drive(self, voltage: volts) -> None:
        self.flywheel.set_control(VoltageOut(voltage).with_enable_foc(True))

    # Tell SysId how to record a frame of data for each motor on the mechanism being
    # characterized.
    @typing.override
    def log(self, sys_id_routine: sysid.SysIdRoutineLog) -> None:
        (
            sys_id_routine.motor("leader")
            .voltage(self.flywheel.get_motor_voltage().value)
            .position(self.flywheel.get_position().value)
            .velocity(self.flywheel.get_velocity().value)
        )
        for motor in self.followers:
            (
                sys_id_routine.motor(f"follower-{motor.device_id}")
                .voltage(motor.get_motor_voltage().value)
                .position(motor.get_position().value)
                .velocity(motor.get_velocity().value)
            )
