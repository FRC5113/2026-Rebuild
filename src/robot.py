import math
from pathlib import Path

import wpilib
from magicbot import feedback
from phoenix6 import CANBus
from phoenix6.hardware import CANcoder, Pigeon2, TalonFX, TalonFXS
from robotpy_apriltag import AprilTagFieldLayout
from wpilib import DigitalInput, DriverStation, DutyCycleEncoder, Field2d
from wpimath import units
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Rotation3d, Transform3d

from autonomous.auto_base import AutoBase
from components.drive_control import DriveControl
from components.intake import Intake
from components.odometry import Odometry
from components.shooter import Shooter
from components.shooter_controller import ShooterController
from components.swerve_drive import SwerveDrive
from components.swerve_wheel import SwerveWheel
from components.sysid_drive import SysIdDriveLinear
from lemonlib import LemonCamera, LemonInput, LemonRobot
from lemonlib.smart import SmartPreference, SmartProfile
from lemonlib.util import (
    AlertManager,
    AlertType,
    curve,
)


class MyRobot(LemonRobot):
    sysid_drive: SysIdDriveLinear
    shooter_controller: ShooterController
    drive_control: DriveControl
    odometry: Odometry

    swerve_drive: SwerveDrive
    front_left: SwerveWheel
    front_right: SwerveWheel
    rear_left: SwerveWheel
    rear_right: SwerveWheel

    shooter: Shooter

    # greatest speed that chassis should move (not greatest possible speed)
    top_speed = SmartPreference(3.0)
    top_omega = SmartPreference(6.0)

    rasing_slew_rate: SmartPreference = SmartPreference(5.0)
    falling_slew_rate: SmartPreference = SmartPreference(5.0)
    intake: Intake

    def createObjects(self):
        """This method is where all attributes to be injected are
        initialized. This is done here rather that inside the components
        themselves so that all constants and initialization parameters
        can be found in one place. Also, attributes shared by multiple
        components, such as the NavX, need only be created once.
        """
        self.tuning_enabled = True

        self.canivore_canbus = CANBus("can0")
        self.rio_canbus = CANBus.roborio()

        """
        SWERVE
        """
        # hardware
        self.front_left_speed_motor = TalonFX(11, self.canivore_canbus)
        self.front_left_direction_motor = TalonFX(12, self.canivore_canbus)
        self.front_left_cancoder = CANcoder(13, self.canivore_canbus)

        self.front_right_speed_motor = TalonFX(21, self.canivore_canbus)
        self.front_right_direction_motor = TalonFX(22, self.canivore_canbus)
        self.front_right_cancoder = CANcoder(23, self.canivore_canbus)

        self.rear_left_speed_motor = TalonFX(41, self.canivore_canbus)
        self.rear_left_direction_motor = TalonFX(42, self.canivore_canbus)
        self.rear_left_cancoder = CANcoder(43, self.canivore_canbus)

        self.rear_right_speed_motor = TalonFX(31, self.canivore_canbus)
        self.rear_right_direction_motor = TalonFX(32, self.canivore_canbus)
        self.rear_right_cancoder = CANcoder(33, self.canivore_canbus)

        # physical constants
        self.offset_x: units.meters = 0.28575
        self.offset_y: units.meters = 0.28575

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
                "kS": 0.17,
                "kV": 0.104,
                "kA": 0.01,
            },
            (not self.low_bandwidth) and self.tuning_enabled,
        )
        self.direction_profile = SmartProfile(
            "direction",
            {
                "kP": 69.235,
                "kI": 0.0,
                "kD": 6.6971,
                "kS": 0.14,
                "kV": 0.375,
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
        INTAKE
        """

        self.intake_spin_motor = TalonFX(51)
        self.intake_left_motor = TalonFX(52)
        self.intake_right_motor = TalonFX(53)
        self.intake_left_encoder = DutyCycleEncoder(DigitalInput(0))
        self.intake_right_encoder = DutyCycleEncoder(DigitalInput(1))

        self.intake_spin_amps: units.amperes = 40.0
        self.intake_arm_amps: units.amperes = 20.0

        self.intake_profile = SmartProfile(
            "intake",
            {
                "kP": 0.0,
                "kI": 0.0,
                "kD": 0.0,
                "kS": 0.0,
                "kV": 0.0,
                "kG": 0.0,
                "kMaxV": 0.0,
                "kMaxA": 0.0,
            },
            (not self.low_bandwidth) and self.tuning_enabled,
        )

        """
        SHOOTER
        """
        self.shooter_left_motor = TalonFX(2, self.rio_canbus)
        self.shooter_right_motor = TalonFX(3, self.rio_canbus)

        self.shooter_gear_ratio = 1.0
        self.shooter_amps: units.amperes = 60.0

        self.shooter_profile = SmartProfile(
            "shooter",
            {
                "kP": 0.002,
                "kI": 0.0,
                "kD": 0.0,
                "kS": 0.0,
                "kV": 0.11137,
                "kA": 0.29663,
            },
            (not self.low_bandwidth) and self.tuning_enabled,
        )

        """
        INDEXER
        """
        self.shooter_left_kicker_motor = TalonFXS(4, self.rio_canbus)
        self.shooter_right_kicker_motor = TalonFXS(5, self.rio_canbus)
        self.shooter_kicker_amps: units.amperes = 20.0
        self.shooter_conveyor_amps: units.amperes = 10.0
        """
        ODOMETRY
        """
        # Custom apriltag field layout
        self.field_layout = AprilTagFieldLayout(
            str(Path(__file__).parent.resolve() / "2026_test_field.json")
        )

        # self.field_layout = AprilTagFieldLayout.loadField(
        #     AprilTagField.k2026RebuiltWelded
        # )

        # Robot to Camera Transforms
        self.rtc_front_left = Transform3d(
            -self.offset_x, self.offset_y, 0.0, Rotation3d(0, 30, 45)
        )
        self.rtc_front_right = Transform3d(
            self.offset_x, self.offset_y, 0.0, Rotation3d(0, 30, -45)
        )
        self.rtc_back_left = Transform3d(
            -self.offset_x, -self.offset_y, 0.0, Rotation3d(0, 30, 135)
        )
        self.rtc_back_right = Transform3d(
            self.offset_x, -self.offset_y, 0.0, Rotation3d(0, 30, -135)
        )
        self.temp_cam = Transform3d(-0.31115, 0.0, 0.5715, Rotation3d())

        self.camera_front_left = LemonCamera(
            "Front_Left", self.temp_cam, self.field_layout
        )
        # self.camera_front_right = LemonCamera(
        #     "Front_Right", self.rtc_front_right, self.field_layout
        # )
        # self.camera_back_left = LemonCamera(
        #     "Back_Left", self.rtc_back_left, self.field_layout
        # )
        # self.camera_back_right = LemonCamera(
        #     "Back_Right", self.rtc_back_right, self.field_layout
        # )

        """
        MISCELLANEOUS
        """

        self.pigeon = Pigeon2(30, self.canivore_canbus)

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

        self.estimated_field = Field2d()

        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            self.alliance = True
        else:
            self.alliance = False

    def enabledperiodic(self):
        self.drive_control.engage()
        # self.shooter_controller.engage()

    def autonomousPeriodic(self):
        self._display_auto_trajectory()

    def teleopInit(self):
        print("Teleop Init")
        # initialize HIDs here in case they are changed after robot initializes
        self.primary = LemonInput(0)
        self.secondary = LemonInput(1)

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
        # Cache inputs called multiple times
        primary_r2 = self.primary.getR2Axis()
        primary_l2 = self.primary.getL2Axis()
        primary_ly = self.primary.getLeftY()
        primary_lx = self.primary.getLeftX()
        primary_rx = self.primary.getRightX()
        primary_ry = self.primary.getRightY()

        """
        SWERVE
        """
        with self.consumeExceptions():
            rotate_mult = 0.75

            # if both 25% else 50 or 75
            if (primary_r2 >= 0.8) and (primary_l2 >= 0.8):
                mult = 0.25
            elif primary_r2 >= 0.8:
                mult = 0.75
            elif primary_l2 >= 0.8:
                mult = 0.5
            else:
                mult = 1.0

            # # only apply the curve and slew rate if the input is above the deadband, otherwise set to 0 to avoid useless math
            # if abs(primary_ly) <= 0.0:
            #     vx = 0.0
            # else:
            #     vx = self.x_filter.calculate(
            #         self.sammi_curve(primary_ly) * mult * self.top_speed
            #     )
            # if abs(primary_lx) <= 0.0:
            #     vy = 0.0
            # else:
            #     vy = self.y_filter.calculate(
            #         self.sammi_curve(primary_lx) * mult * self.top_speed
            #     )
            # if abs(primary_rx) <= 0.0:
            #     omega = 0.0
            # else:
            #     omega = self.theta_filter.calculate(
            #         -self.sammi_curve(primary_rx) * rotate_mult * self.top_omega
            #     )

            # self.drive_control.drive_manual(
            #     vx,
            #     -vy,
            #     -omega,
            #     not self.primary.getCreateButton(),  # temporary
            # )

            # if self.primary.getSquareButton():
            #     self.swerve_drive.reset_gyro()
            # self.swerve_drive.doTelemetry()

        """
        INTAKE
        """
        with self.consumeExceptions():
            if self.secondary.getLeftBumper():
                self.intake.set_voltage(0.5)

        """
        SHOOTER
        """
        with self.consumeExceptions():
            if self.secondary.getStartButton():
                self.shooter.set_velocity(10)
            if self.secondary.getOptionsButton():
                self.shooter.set_velocity(40)
            if self.secondary.getAButton():
                self.shooter.set_velocity(43)
            if self.secondary.getBButton():
                self.shooter.set_velocity(45)
            if self.secondary.getXButton():
                self.shooter.set_velocity(47)
            if self.secondary.getYButton():
                self.shooter.set_velocity(50)

            if self.secondary.getRightTriggerAxis() >= 0.8:
                self.shooter.set_kicker_voltage(8)

    def disabledPeriodic(self):
        # make magicbot happy
        pass

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
