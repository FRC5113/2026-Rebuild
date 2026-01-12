import math

import wpilib
from wpilib import (
    Field2d,
    RobotController,
    DriverStation,
    PowerDistribution,
)
from wpilib import RobotController
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout

from wpimath import units
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Transform3d, Rotation3d

from phoenix6.hardware import CANcoder, TalonFX, Pigeon2
from phoenix6 import CANBus

from magicbot import feedback

from lemonlib import LemonInput
from lemonlib.util import (
    curve,
    AlertManager,
    AlertType,
)
from lemonlib.smart import SmartPreference, SmartProfile
from lemonlib import LemonRobot, LemonCamera
from lemonlib.util import AsymmetricSlewLimiter

from autonomous.auto_base import AutoBase
from components.swerve_drive import SwerveDrive
from components.swerve_wheel import SwerveWheel
from components.drive_control import DriveControl
from components.sysid_drive import SysIdDriveLinear


class MyRobot(LemonRobot):
    sysid_drive: SysIdDriveLinear
    drive_control: DriveControl

    swerve_drive: SwerveDrive
    front_left: SwerveWheel
    front_right: SwerveWheel
    rear_left: SwerveWheel
    rear_right: SwerveWheel

    # greatest speed that chassis should move (not greatest possible speed)
    top_speed = SmartPreference(3.0)
    top_omega = SmartPreference(6.0)

    rasing_slew_rate = SmartPreference(5.0)
    falling_slew_rate = SmartPreference(5.0)

    tuning_enabled = True

    def createObjects(self):
        """This method is where all attributes to be injected are
        initialized. This is done here rather that inside the components
        themselves so that all constants and initialization parameters
        can be found in one place. Also, attributes shared by multiple
        components, such as the NavX, need only be created once.
        """

        self.canicore_canbus = CANBus("can0")

        """
        SWERVE
        """

        # hardware
        self.front_left_speed_motor = TalonFX(11, self.canicore_canbus)
        self.front_left_direction_motor = TalonFX(12, self.canicore_canbus)
        self.front_left_cancoder = CANcoder(13, self.canicore_canbus)

        self.front_right_speed_motor = TalonFX(21, self.canicore_canbus)
        self.front_right_direction_motor = TalonFX(22, self.canicore_canbus)
        self.front_right_cancoder = CANcoder(23, self.canicore_canbus)

        self.rear_left_speed_motor = TalonFX(41, self.canicore_canbus)
        self.rear_left_direction_motor = TalonFX(42, self.canicore_canbus)
        self.rear_left_cancoder = CANcoder(43, self.canicore_canbus)

        self.rear_right_speed_motor = TalonFX(31, self.canicore_canbus)
        self.rear_right_direction_motor = TalonFX(32, self.canicore_canbus)
        self.rear_right_cancoder = CANcoder(33, self.canicore_canbus)

        # physical constants
        self.offset_x: units.meters = 0.3175
        self.offset_y: units.meters = 0.3175
        self.drive_gear_ratio = 6.75
        self.direction_gear_ratio = 150 / 7
        self.wheel_radius: units.meters = 0.0508
        self.max_speed: units.meters_per_second = 4.7
        self.direction_amps: units.amperes = 40.0
        self.speed_amps: units.amperes = 60.0

        # profiles
        self.speed_profile = SmartProfile(
            "speed",
            {
                "kP": 0.0,
                "kI": 0.0,
                "kD": 0.0,
                "kS": 0.0,
                "kV": 0.0,
                "kA": 0.0,
            },
            (not self.low_bandwidth) and self.tuning_enabled,
        )
        self.direction_profile = SmartProfile(
            "direction",
            {
                "kP": 0.0,
                "kI": 0.0,
                "kD": 0.0,
                "kS": 0.0,
                "kV": 0.0,
                "kA": 0.0,
            },
            (not self.low_bandwidth) and self.tuning_enabled,
        )
        self.translation_profile = SmartProfile(
            "translation",
            {
                "kP": 1.0,
                "kI": 0.0,
                "kD": 0.0,
            },
            (not self.low_bandwidth) and self.tuning_enabled,
        )
        self.rotation_profile = SmartProfile(
            "rotation",
            {
                "kP": 0.0,
                "kI": 0.0,
                "kD": 0.0,
                "kMaxV": 10.0,
                "kMaxA": 100.0,
                "kMinInput": -math.pi,
                "kMaxInput": math.pi,
            },
            (not self.low_bandwidth) and self.tuning_enabled,
        )

        """
        MISCELLANEOUS
        """

        self.pigeon = Pigeon2(30, self.canicore_canbus)

        self.fms = DriverStation.isFMSAttached()

        # driving curve
        self.sammi_curve = curve(
            lambda x: 1.89 * x**3 + 0.61 * x, 0.0, deadband=0.1, max_mag=1.0
        )

        # alerts
        AlertManager(self.logger)
        if self.low_bandwidth:
            AlertManager.instant_alert(
                "Low Bandwidth Mode is active! Tuning is disabled.", AlertType.INFO
            )

        self.pdh = PowerDistribution()

        self.estimated_field = Field2d()

        self.field_layout = AprilTagFieldLayout.loadField(AprilTagField.kDefaultField)

        self.robot_to_camera_front = Transform3d(0.0, 0.0, 0.0, Rotation3d())

        self.camera_front = LemonCamera(
            "Global_Shutter_Camera", self.robot_to_camera_front, self.field_layout
        )

        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            self.alliance = True
        else:
            self.alliance = False

    def enabledperiodic(self):
        self.drive_control.engage()

    def autonomousPeriodic(self):
        self._display_auto_trajectory()

    def teleopInit(self):
        # initialize HIDs here in case they are changed after robot initializes
        self.primary = LemonInput(0, "PS5")

        self.x_filter = SlewRateLimiter(
            self.rasing_slew_rate  # , self.falling_slew_rate
        )
        self.y_filter = SlewRateLimiter(
            self.rasing_slew_rate  # , self.falling_slew_rate
        )
        self.theta_filter = SlewRateLimiter(
            self.rasing_slew_rate  # , self.falling_slew_rate
        )

    def teleopPeriodic(self):
        with self.consumeExceptions():

            """
            SWERVE
            """
            rotate_mult = 0.75
            mult = 1
            if self.primary.getR2Axis() >= 0.8:
                mult *= 0.5
            if self.primary.getL2Axis() >= 0.8:
                mult *= 0.5

            self.drive_control.drive_manual(
                self.x_filter.calculate(
                    self.sammi_curve(self.primary.getPovX()) * mult * self.top_speed
                ),
                self.y_filter.calculate(
                    self.sammi_curve(self.primary.getPovY()) * mult * self.top_speed
                ),
                self.theta_filter.calculate(
                    -self.sammi_curve(self.primary.getRightX())
                    * rotate_mult
                    * self.top_omega
                ),
                not self.primary.getCreateButton(),  # temporary
            )
            if self.primary.getSquareButton():
                self.swerve_drive.reset_gyro()

    @feedback
    def get_voltage(self) -> units.volts:
        return RobotController.getBatteryVoltage()

    def _display_auto_trajectory(self) -> None:
        selected_auto = self._automodes.chooser.getSelected()
        if isinstance(selected_auto, AutoBase):
            selected_auto.display_trajectory()

    @feedback
    def display_auto_state(self) -> None:
        selected_auto = self._automodes.chooser.getSelected()
        if isinstance(selected_auto, AutoBase):
            return selected_auto.current_state
        return "No Auto Selected"


if __name__ == "__main__":
    wpilib.run(MyRobot)
