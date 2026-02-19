import math
import typing
from typing import Callable

from phoenix6 import BaseStatusSignal, CANBus
from phoenix6.configs import (
    ClosedLoopGeneralConfigs,
    FeedbackConfigs,
    TalonFXConfiguration,
)
from phoenix6.configs.config_groups import NeutralModeValue
from phoenix6.controls import VoltageOut
from phoenix6.hardware import CANcoder, TalonFX
from phoenix6.signals import (
    FeedbackSensorSourceValue,
    NeutralModeValue,
)
from phoenix6.status_code import StatusCode
from wpilib import sysid
from wpimath.units import volts

from subsystems.sysid_subsystem import SysidSubsystem

from . import tryUntilOk


class SwerveModule(SysidSubsystem):
    def __init__(self) -> None:
        super().__init__()
        self.canbus = CANBus("can0")
        self.steer = TalonFX(32, self.canbus)
        self.encoder = CANcoder(33, self.canbus)

        self.steer.get_fault_field().set_update_frequency(
            frequency_hz=4, timeout_seconds=0.01
        )
        self.steer.get_closed_loop_error().set_update_frequency(20)
        self.steer.get_closed_loop_reference().set_update_frequency(20)
        self.steer.get_closed_loop_output().set_update_frequency(20)

        self.steer_configs = TalonFXConfiguration()

        # current limits
        self.steer_configs.current_limits.stator_current_limit = 40.0

        self.steer_configs.motor_output.neutral_mode = NeutralModeValue.BRAKE

        # feedback configs - uses fused CANcoder for direction motor
        self.steer_configs.feedback = (
            FeedbackConfigs()
            .with_feedback_remote_sensor_id(self.encoder.device_id)
            .with_feedback_sensor_source(
                FeedbackSensorSourceValue.FUSED_CANCODER
            )  # Fuse CANcoder with internal encoder for better accuracy
            .with_rotor_to_sensor_ratio(
                150.0 / 7.0
            )  # Converts motor rotations to sensor rotations
        )

        # Enable continuous wrap so direction motor can take shortest path to target angle
        self.steer_configs.closed_loop_general = (
            ClosedLoopGeneralConfigs().with_continuous_wrap(True)
        )
        tryUntilOk(
            5,
            lambda: self.steer.configurator.apply(self.steer_configs),
        )

        self.direction_position = self.steer.get_position()
        self.direction_velocity = self.steer.get_velocity()

        self.signals = [
            self.direction_position,
            self.direction_velocity,
        ]

        self.meters_per_wheel_rotation = 0.0508 * math.tau
        self.drive_rot_per_meter = 1.0 / self.meters_per_wheel_rotation

        BaseStatusSignal.set_update_frequency_for_all(250, self.signals)

    def tryUntilOk(attempts: int, command: Callable[[], StatusCode]):
        for _ in range(attempts):
            code = command()
            if code.is_ok():
                break

    @typing.override
    def log(self, sys_id_routine: sysid.SysIdRoutineLog) -> None:
        # Record a frame for the left motors.  Since these share an encoder, we consider
        # the entire group to be one motor.
        sys_id_routine.motor("steer").voltage(
            self.steer.get_motor_voltage().value
        ).position(self.steer.get_position().value).velocity(
            self.steer.get_velocity().value
        )

    @typing.override
    def drive(self, voltage: volts) -> None:
        BaseStatusSignal.refresh_all(self.signals)
        steer_request = VoltageOut(voltage).with_enable_foc(True)
        self.steer.set_control(steer_request)
