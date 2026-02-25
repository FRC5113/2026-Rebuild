import typing

import phoenix6
from phoenix6.configs import (
    FeedbackConfigs,
    MotorOutputConfigs,
    SoftwareLimitSwitchConfigs,
    TalonFXConfiguration,
)
from phoenix6.controls import Follower, VoltageOut
from phoenix6.signals import (
    InvertedValue,
    MotorAlignmentValue,
    NeutralModeValue,
)
from wpilib import sysid
from wpimath.units import volts

from subsystems.sysid_subsystem import SysidSubsystem

FollowerDescriptor = tuple[phoenix6.hardware.TalonFX, bool]


class Arm(SysidSubsystem):
    """SysId subsystem for an arm mechanism (single-jointed).

    The arm uses BRAKE mode by default and logs position in rotations
    (mechanism-side) so SysId can fit kS, kG, kV, and kA for an arm model.
    """

    def __init__(
        self,
        arm_motor: phoenix6.hardware.TalonFX,
        *followers: FollowerDescriptor,
        gearing: float,
        inverted: bool = False,
        forward_limit_rotations: float | None = None,
        reverse_limit_rotations: float | None = None,
        name: str | None = None,
    ) -> None:
        super().__init__()
        self.arm = arm_motor
        self.followers = [motor for motor, _ in followers]
        if name is not None:
            self.setName(name)

        motor_output_config = MotorOutputConfigs()
        motor_output_config.neutral_mode = NeutralModeValue.BRAKE
        if inverted:
            motor_output_config.inverted = InvertedValue.CLOCKWISE_POSITIVE

        feedback_config = FeedbackConfigs().with_sensor_to_mechanism_ratio(gearing)

        arm_config = (
            TalonFXConfiguration()
            .with_feedback(feedback_config)
            .with_motor_output(motor_output_config)
        )

        # Optional soft limits to protect the arm from over-travel
        if forward_limit_rotations is not None or reverse_limit_rotations is not None:
            soft_limits = SoftwareLimitSwitchConfigs()
            if forward_limit_rotations is not None:
                soft_limits.forward_soft_limit_enable = True
                soft_limits.forward_soft_limit_threshold = forward_limit_rotations
            if reverse_limit_rotations is not None:
                soft_limits.reverse_soft_limit_enable = True
                soft_limits.reverse_soft_limit_threshold = reverse_limit_rotations
            arm_config = arm_config.with_software_limit_switch(soft_limits)

        self.arm.configurator.apply(arm_config)

        for motor, oppose_leader in followers:
            motor.configurator.apply(
                TalonFXConfiguration()
                .with_motor_output(motor_output_config)
                .with_feedback(feedback_config)
            )
            motor.set_control(
                Follower(
                    self.arm.device_id,
                    (
                        MotorAlignmentValue.OPPOSED
                        if oppose_leader
                        else MotorAlignmentValue.ALIGNED
                    ),
                )
            )

    @typing.override
    def drive(self, voltage: volts) -> None:
        self.arm.set_control(VoltageOut(voltage).with_enable_foc(True))

    @typing.override
    def log(self, sys_id_routine: sysid.SysIdRoutineLog) -> None:
        (
            sys_id_routine.motor("leader")
            .voltage(self.arm.get_motor_voltage().value)
            .position(self.arm.get_position().value)
            .velocity(self.arm.get_velocity().value)
        )
        for motor in self.followers:
            (
                sys_id_routine.motor(f"follower-{motor.device_id}")
                .voltage(motor.get_motor_voltage().value)
                .position(motor.get_position().value)
                .velocity(motor.get_velocity().value)
            )
