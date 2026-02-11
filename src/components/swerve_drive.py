import math

from choreo.trajectory import SwerveSample
from magicbot import feedback, will_reset_to
from phoenix6 import BaseStatusSignal
from phoenix6.hardware import Pigeon2
from wpilib import DriverStation, SmartDashboard
from wpilib.sysid import SysIdRoutineLog
from wpimath import units
from wpimath.controller import HolonomicDriveController
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveDrive4Kinematics,
    SwerveModulePosition,
    SwerveModuleState,
)
from wpiutil import Sendable, SendableBuilder

from components.swerve_wheel import SwerveWheel
from lemonlib.smart import SmartController, SmartProfile
from lemonlib.util import Alert, AlertType


class SwerveDrive(Sendable):
    # Distance from robot center to wheel in X and Y directions
    offset_x: units.meters
    offset_y: units.meters
    drive_gear_ratio: float
    wheel_radius: units.meters
    max_speed: units.meters_per_second
    front_left: SwerveWheel
    front_right: SwerveWheel
    rear_left: SwerveWheel
    rear_right: SwerveWheel
    pigeon: Pigeon2  # IMU/gyroscope for heading measurement
    translation_profile: SmartProfile
    rotation_profile: SmartProfile

    # will_reset_to ensures these values reset to defaults each robot loop iteration
    translationX = will_reset_to(0)
    translationY = will_reset_to(0)
    rotationX = will_reset_to(0)
    field_relative = will_reset_to(True)
    has_desired_pose = will_reset_to(False)

    doing_sysid = will_reset_to(False)
    sysid_rot = will_reset_to(False)
    sysid_volts = will_reset_to(0.0)

    def __init__(self) -> None:
        Sendable.__init__(self)

    @staticmethod
    def shouldFlipPath():
        # Boolean supplier that controls when the path will be mirrored for the red alliance
        # This will flip the path being followed to the red side of the field.
        # THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed

    """
    INITIALIZATION METHODS
    """

    def setup(self) -> None:
        """
        This function is automatically called after the components have
        been injected.
        """
        # Define wheel positions relative to robot center (using standard WPILib coordinate system)
        # Positive X is forward, positive Y is left
        self.front_left_pose = Translation2d(-self.offset_x, self.offset_y)
        self.front_right_pose = Translation2d(self.offset_x, self.offset_y)
        self.rear_left_pose = Translation2d(-self.offset_x, -self.offset_y)
        self.rear_right_pose = Translation2d(self.offset_x, -self.offset_y)
        # Kinematics converts between chassis speeds and individual module states
        self.kinematics = SwerveDrive4Kinematics(
            self.front_left_pose,
            self.front_right_pose,
            self.rear_left_pose,
            self.rear_right_pose,
        )
        self.chassis_speeds = ChassisSpeeds()
        # Pre-compute stopped states to avoid recalculating when robot is still
        self.still_states = self.kinematics.toSwerveModuleStates(self.chassis_speeds)
        self.swerve_module_states = self.still_states
        SmartDashboard.putData("Swerve Drive", self)

        # Pose estimator fuses odometry with vision for more accurate position tracking
        self.pose_estimator = SwerveDrive4PoseEstimator(
            self.kinematics,
            Rotation2d(),
            (
                SwerveModulePosition(),
                SwerveModulePosition(),
                SwerveModulePosition(),
                SwerveModulePosition(),
            ),
            Pose2d(),
        )
        self.period = 0.02  # Default loop period in seconds (50Hz)

        self.desired_pose = Pose2d()
        self.starting_pose = None  # only used in sim

        self.pigeon_offset = Rotation2d()  # Allows software adjustment of gyro heading
        self.pigeon_alert = Alert(
            "Pigeon heading has been reset.", AlertType.INFO, timeout=3.0
        )

        self.modules = (
            self.front_left,
            self.front_right,
            self.rear_left,
            self.rear_right,
        )

        self.module_positions = [None] * 4

        self.cached_yaw = 0.0
        self.cached_yaw_rate = 0.0

        # 4 signals per module + yaw + yaw rate
        self.all_signals = []

        for module in self.modules:
            self.all_signals.extend(module.getSignals())

        self.all_signals.append(self.pigeon.get_yaw())

        BaseStatusSignal.set_update_frequency_for_all(250, self.all_signals)

    def initSendable(self, builder: SendableBuilder) -> None:
        # Configure data sent to SmartDashboard's swerve widget
        builder.setSmartDashboardType("SwerveDrive")
        builder.addDoubleProperty(
            "Robot Angle",
            # Rotate to match field widget
            lambda: self.pigeon.getRotation2d().degrees(),
            lambda _: None,
        )
        # Speed multiplied by 2 to scale for dashboard display
        builder.addDoubleProperty(
            "Front Left Velocity",
            lambda: self.swerve_module_states[0].speed * 2,
            lambda _: None,
        )
        builder.addDoubleProperty(
            "Front Left Angle",
            lambda: self.swerve_module_states[0].angle.degrees(),
            lambda _: None,
        )
        builder.addDoubleProperty(
            "Front Right Velocity",
            lambda: self.swerve_module_states[1].speed * 2,
            lambda _: None,
        )
        builder.addDoubleProperty(
            "Front Right Angle",
            lambda: self.swerve_module_states[1].angle.degrees(),
            lambda _: None,
        )
        builder.addDoubleProperty(
            "Back Left Velocity",
            lambda: self.swerve_module_states[2].speed * 2,
            lambda _: None,
        )
        builder.addDoubleProperty(
            "Back Left Angle",
            lambda: self.swerve_module_states[2].angle.degrees(),
            lambda _: None,
        )
        builder.addDoubleProperty(
            "Back Right Velocity",
            lambda: self.swerve_module_states[3].speed * 2,
            lambda _: None,
        )
        builder.addDoubleProperty(
            "Back Right Angle",
            lambda: self.swerve_module_states[3].angle.degrees(),
            lambda _: None,
        )

    def on_enable(self):
        # Create PID controllers for X/Y translation and rotation
        self.x_controller = self.translation_profile.create_wpi_pid_controller()
        self.y_controller = self.translation_profile.create_wpi_pid_controller()
        self.theta_controller = (
            self.rotation_profile.create_wpi_profiled_pid_controller_radians()
        )
        # HolonomicDriveController combines X, Y, and theta control for autonomous driving
        self.holonomic_controller = HolonomicDriveController(
            self.x_controller, self.y_controller, self.theta_controller
        )
        # Allow theta controller to wrap around from -pi to pi (continuous rotation)
        self.theta_controller.enableContinuousInput(-math.pi, math.pi)
        self.smart_theta_controller = SmartController(
            "Theta Controller", self.theta_controller.calculate, True
        )

    """
    INFORMATIONAL METHODS
    """

    def get_estimated_pose(self) -> Pose2d:
        return self.pose_estimator.getEstimatedPosition()

    def get_velocity(self) -> ChassisSpeeds:
        return self.chassis_speeds

    def get_module_states(
        self,
    ) -> tuple[
        SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState
    ]:
        return (
            self.front_left.getMeasuredState(),
            self.front_right.getMeasuredState(),
            self.rear_left.getMeasuredState(),
            self.rear_right.getMeasuredState(),
        )

    @feedback
    def get_distance_from_desired_pose(self) -> units.meters:
        if not self.has_desired_pose:
            return 0
        return self.desired_pose.translation().distance(
            self.get_estimated_pose().translation()
        )

    def get_distance_from_pose(self, pose: Pose2d) -> units.meters:
        return pose.translation().distance(self.get_estimated_pose().translation())

    def at_angle(self) -> bool:
        if not self.has_desired_pose:
            return True
        angle_error = abs(
            (
                self.desired_pose.rotation() - self.get_estimated_pose().rotation()
            ).degrees()
        )
        return angle_error < 3.0  # degrees of tolerance

    """
    CONTROL METHODS
    """

    def drive(
        self,
        translationX: units.meters_per_second,
        translationY: units.meters_per_second,
        rotationX: units.radians_per_second,
        field_relative: bool,
        period: units.seconds,
    ):
        # Store drive commands to be processed in execute()
        self.translationX = translationX
        self.translationY = translationY
        self.rotationX = rotationX
        self.period = period
        self.field_relative = field_relative

    def sysid_drive(self, volts: float, rot: float = 0.0) -> None:
        # System identification mode for characterizing drive motors
        self.doing_sysid = True
        self.sysid_volts = volts

    def sysid_rot(self, volts: float, rot: float = 0.0) -> None:
        # System identification mode for characterizing drive motors
        self.doing_sysid = True
        self.sysid_volts = volts
        self.sysid_rotate = True

    def set_desired_pose(self, pose: Pose2d):
        self.desired_pose = pose
        self.has_desired_pose = True

    def reset_gyro(self) -> None:
        self.pigeon.set_yaw(0)
        self.pigeon_alert.enable()

    def addVisionPoseEstimate(self, pose: Pose2d, timestamp: units.seconds):
        self.pose_estimator.addVisionMeasurement(pose, timestamp)

    def set_pigeon_offset(self, offset: units.degrees):
        """set angle added to reading from pigeon"""
        self.pigeon_offset = Rotation2d.fromDegrees(offset)

    def follow_trajectory(self, sample: SwerveSample):
        # Follow a Choreo trajectory sample using feedforward + feedback control
        pose = self.get_estimated_pose()

        # Combine trajectory feedforward velocities with PID feedback corrections
        speeds = ChassisSpeeds(
            sample.vx + self.x_controller.calculate(pose.X(), sample.x),
            sample.vy + self.y_controller.calculate(pose.Y(), sample.y),
            sample.omega
            + self.theta_controller.calculate(
                pose.rotation().radians(), sample.heading
            ),
        )
        self.drive(speeds.vx, speeds.vy, speeds.omega, False, self.period)

    def point_towards_joy(self, rightX: float, rightY: float):
        # Convert joystick input to rotation command for pointing robot
        moved = abs(rightX) > 0.1 or abs(rightY) > 0.1  # Deadband check
        if not moved:
            return 0.0
        # Convert joystick position to angle, offset by 90 degrees to align with robot forward
        angle = math.atan2(rightY, rightX) - math.radians(90)
        current_angle = math.radians(self.pigeon.get_yaw().value)
        output = self.smart_theta_controller.calculate(current_angle, angle)
        return output

    def point_towards(self, angle: units.radians):
        current_angle = math.radians(self.pigeon.get_yaw().value)
        output = self.smart_theta_controller.calculate(current_angle, angle)
        self.translationX = 0.0
        self.translationY = 0.0
        self.rotationX = output

    def driveRobotRelative(self, speeds: ChassisSpeeds):
        """Drives the robot using ROBOT RELATIVE speeds.
        This is used for path following."""
        return self.drive(
            speeds.vx,
            speeds.vy,
            speeds.omega,
            False,  # Robot relative, not field relative
            self.period,
        )

    def set_starting_pose(self, pose: Pose2d):
        """ONLY USE IN SIM!"""
        self.starting_pose = pose
        if pose is not None:
            self.pose_estimator.resetPose(pose)

    def resetPose(self):
        self.pose_estimator.resetPose(Pose2d())

    """
    TELEMETRY METHODS
    """

    def sendAdvantageScopeData(self):
        """Put swerve module setpoints and measurements to NT.
        This is used mainly for AdvantageScope's swerve tab"""
        # Format: [angle1, speed1, angle2, speed2, ...] for each module
        swerve_setpoints = []
        for state in self.swerve_module_states:
            swerve_setpoints += [state.angle.degrees(), state.speed]
        SmartDashboard.putNumberArray("Swerve Setpoints", swerve_setpoints)
        swerve_measurements = []
        swerve_measurements += self.front_left.getMeasuredState()
        swerve_measurements += self.front_right.getMeasuredState()
        swerve_measurements += self.rear_left.getMeasuredState()
        swerve_measurements += self.rear_right.getMeasuredState()
        SmartDashboard.putNumberArray("Swerve Measurements", swerve_measurements)

    """
    sys-id
    """

    # Tell SysId how to record a frame of data for each motor on the mechanism being
    # characterized.
    def log(self, sys_id_routine: SysIdRoutineLog) -> None:
        # Log voltage, position, and velocity for each drive motor (used for system identification)
        for name, module in zip(("fl", "fr", "rl", "rr"), self.modules):
            sys_id_routine.motor(f"swerve/drive/{name}").voltage(
                module.getVoltage()
            ).position(module.cached_drive_rot / module.drive_rot_per_meter).velocity(
                module.getVelocity()
            )

    def doTelemetry(self):
        self.front_left.putTelem()
        self.front_right.putTelem()
        self.rear_left.putTelem()
        self.rear_right.putTelem()

    """
    EXECUTE
    """

    def execute(self) -> None:
        BaseStatusSignal.refresh_all(self.all_signals)
        self.cached_yaw = BaseStatusSignal.get_latency_compensated_value(
            self.pigeon.get_yaw(), self.pigeon.get_angular_velocity_z_world()
        )
        self.cached_yaw_rate = self.pigeon.get_angular_velocity_z_world().value

        self.sendAdvantageScopeData()

        for i, module in enumerate(self.modules):
            self.module_positions[i] = module.getPosition()

        chassis_rot = Rotation2d(self.cached_yaw) + self.pigeon_offset

        self.pose_estimator.update(
            chassis_rot,
            tuple(self.module_positions),
        )

        # If we have a target pose and aren't close enough (>2cm), use holonomic controller
        if self.has_desired_pose and self.get_distance_from_desired_pose() > 0.02:
            self.chassis_speeds = self.holonomic_controller.calculate(
                self.get_estimated_pose(),
                self.desired_pose,
                0.0,  # Desired linear velocity at target (0 = stop)
                self.desired_pose.rotation(),
            )
        else:
            # Normal teleop driving
            if self.translationX == self.translationY == self.rotationX == 0:
                # Robot is commanded to stop - skip motor commands to reduce CAN usage
                self.swerve_module_states = self.still_states  # Keep NT updated
                self.chassis_speeds = ChassisSpeeds()
                return
            # discretize() compensates for robot rotation during the control period
            # to improve accuracy during fast rotations
            self.chassis_speeds = ChassisSpeeds.discretize(
                (
                    # Convert field-relative commands to robot-relative using gyro heading
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        self.translationX,
                        self.translationY,
                        self.rotationX,
                        chassis_rot,
                    )
                    if self.field_relative
                    else ChassisSpeeds(
                        self.translationX,
                        self.translationY,
                        self.rotationX,
                    )
                ),
                self.period,
            )
        # Convert chassis speeds to individual wheel states
        self.swerve_module_states = self.kinematics.toSwerveModuleStates(
            self.chassis_speeds
        )
        # Scale down all wheel speeds proportionally if any exceed max speed
        self.swerve_module_states = SwerveDrive4Kinematics.desaturateWheelSpeeds(
            self.swerve_module_states,
            self.max_speed,
        )

        # Command each wheel to its target state
        self.front_left.setDesiredState(self.swerve_module_states[0])
        self.front_right.setDesiredState(self.swerve_module_states[1])
        self.rear_left.setDesiredState(self.swerve_module_states[2])
        self.rear_right.setDesiredState(self.swerve_module_states[3])
