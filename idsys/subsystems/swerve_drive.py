import math
import typing

from phoenix6.configs import (
    FeedbackConfigs,
    MotorOutputConfigs,
    Slot0Configs,
    TalonFXConfiguration,
)
from phoenix6.configs.config_groups import NeutralModeValue
from phoenix6.controls import PositionVoltage, VoltageOut
from phoenix6.hardware import CANcoder, TalonFX
from wpilib import sysid
from wpimath.units import volts

from subsystems.sysid_subsystem import SysidSubsystem


class SwerveDrive(SysidSubsystem):
    DRIVE_GEAR_RATIO = 1/((14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0))
    STEER_GEAR_RATIO = 150 / 7

    WHEEL_CIRCUMFERENCE = 4 * 2.54 / 100 * math.pi

    DRIVE_MOTOR_REV_TO_METRES = WHEEL_CIRCUMFERENCE * DRIVE_GEAR_RATIO
    STEER_MOTOR_REV_TO_RAD = math.tau * STEER_GEAR_RATIO

    def __init__(self) -> None:
        super().__init__()
        # The motors on the left side of the drive
        self.drive_1 = TalonFX(11)
        self.drive_2 = TalonFX(21)
        self.drive_3 = TalonFX(41)
        self.drive_4 = TalonFX(31)
        self.drive_motors = [self.drive_1, self.drive_2, self.drive_3, self.drive_4]
        for drive_motor in self.drive_motors:
            drive_gear_ratio_config = FeedbackConfigs().with_sensor_to_mechanism_ratio(
                1 / self.DRIVE_MOTOR_REV_TO_METRES
            )
            drive_config = drive_motor.configurator
            drive_config.apply(
                TalonFXConfiguration().with_feedback(drive_gear_ratio_config)
            )

        self.steer_1 = TalonFX(12)
        self.steer_2 = TalonFX(22)
        self.steer_3 = TalonFX(42)
        self.steer_4 = TalonFX(32)

        self.encoder_1 = CANcoder(13)
        self.encoder_2 = CANcoder(23)
        self.encoder_3 = CANcoder(43)
        self.encoder_4 = CANcoder(33)

        self.steer_motors = [self.steer_1, self.steer_2, self.steer_3, self.steer_4]
        self.steer_encoders = [
            self.encoder_1,
            self.encoder_2,
            self.encoder_3,
            self.encoder_4,
        ]
        for steer_motor, steer_encoder in zip(self.steer_motors, self.steer_encoders):
            steer_motor_config = MotorOutputConfigs()
            steer_motor_config.neutral_mode = NeutralModeValue.BRAKE
            # default to L1 ratio with falcons and then override if we are on l2 with kraken
            steer_pid = (
                Slot0Configs()
                .with_k_p(30.234)
                .with_k_i(0)
                .with_k_d(0.62183)
                .with_k_s(0.1645)
            )

            steer_gear_ratio_config = FeedbackConfigs().with_sensor_to_mechanism_ratio(
                self.STEER_GEAR_RATIO
            )

            steer_motor.set_position(steer_encoder.get_absolute_position().value)

            steer_config = steer_motor.configurator
            steer_config.apply(
                TalonFXConfiguration()
                .with_motor_output(steer_motor_config)
                .with_slot0(steer_pid)
                .with_feedback(steer_gear_ratio_config)
            )

    # Tell SysId how to record a frame of data for each motor on the mechanism being
    # characterized.
    @typing.override
    def log(self, sys_id_routine: sysid.SysIdRoutineLog) -> None:
        # Record a frame for the left motors.  Since these share an encoder, we consider
        # the entire group to be one motor.
        for drive_motor, index in zip(self.drive_motors, range(1, 5)):
            sys_id_routine.motor(f"drive-{index}").voltage(
                drive_motor.get_motor_voltage().value
            ).position(
                drive_motor.get_position().value * self.WHEEL_CIRCUMFERENCE
            ).velocity(
                drive_motor.get_velocity().value * self.WHEEL_CIRCUMFERENCE
            )

    @typing.override
    def drive(self, voltage: volts) -> None:
        steer_request = PositionVoltage(0.0)
        for steer_motor in self.steer_motors:
            steer_motor.set_control(steer_request)
        voltage_request = VoltageOut(voltage)
        for drive_motor in self.drive_motors:
            drive_motor.set_control(voltage_request)