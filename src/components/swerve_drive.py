import math

from choreo.trajectory import SwerveSample
from phoenix6 import configs, hardware, swerve, utils
from phoenix6.signals import StaticFeedforwardSignValue
from phoenix6.swerve import requests
from wpilib import DriverStation, SmartDashboard, Timer
from wpilib.sysid import SysIdRoutineLog
from wpimath import units
from wpimath.controller import HolonomicDriveController
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveDrive4Kinematics,
    SwerveModuleState,
)
from wpiutil import Sendable, SendableBuilder

from generated.tuner_constants import TunerConstants
from lemonlib import fms_feedback
from lemonlib.smart import SmartController, SmartPreference, SmartProfile
from lemonlib.util import Alert, AlertType


class SwerveDrive(Sendable):
    """Swerve drive using the Phoenix 6 Swerve API (SwerveDrivetrain).

    The underlying ``phoenix6.swerve.SwerveDrivetrain`` owns the hardware,
    runs its own high-frequency odometry thread, and applies
    ``SwerveRequest`` objects.
    """

    max_speed: units.meters_per_second
    translation_profile: SmartProfile
    rotation_profile: SmartProfile
    steer_profile: SmartProfile
    drive_profile: SmartProfile
    tuning_enabled: bool

    telemetry_enabled = SmartPreference(False)
    telemetry_period = SmartPreference(0.1)
    adv_scope_enabled = SmartPreference(False)
    adv_scope_period = SmartPreference(0.1)

    def __init__(self) -> None:
        Sendable.__init__(self)
        # The request to apply on the next execute() call.
        # Set by control methods, consumed (and cleared) in execute().
        # NOT will_reset_to — it persists across one cycle boundary so that
        # execute() (which runs before drive_control) can read the request
        # that drive_control set on the previous cycle.
        self._pending_request = None
        # Cached drivetrain state — updated once per execute() cycle.
        # Avoids repeated get_state() calls (each copies C++ → Python objects).
        self._cached_pose = Pose2d()
        self.chassis_speeds = ChassisSpeeds()
        self.swerve_module_states = (
            SwerveModuleState(),
            SwerveModuleState(),
            SwerveModuleState(),
            SwerveModuleState(),
        )

    @staticmethod
    def shouldFlipPath():
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed

    """
    INITIALIZATION
    """

    def setup(self) -> None:
        """Called by MagicBot after attribute injection."""
        tc = TunerConstants

        # Build the Phoenix 6 SwerveDrivetrain — it creates all hardware internally
        self._drivetrain = swerve.SwerveDrivetrain(
            hardware.TalonFX,
            hardware.TalonFX,
            hardware.CANcoder,
            tc.drivetrain_constants,
            [tc.front_left, tc.front_right, tc.back_left, tc.back_right],
        )

        self.kinematics: SwerveDrive4Kinematics = self._drivetrain.kinematics

        # Pre-built SwerveRequest objects (mutated in-place each cycle)
        self._field_centric_req = (
            requests.FieldCentric()
            .with_deadband(0.0)
            .with_rotational_deadband(0.0)
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
            .with_steer_request_type(
                swerve.SwerveModule.SteerRequestType.MOTION_MAGIC_EXPO
            )
        )
        self._robot_centric_req = (
            requests.RobotCentric()
            .with_deadband(0.0)
            .with_rotational_deadband(0.0)
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
            .with_steer_request_type(
                swerve.SwerveModule.SteerRequestType.MOTION_MAGIC_EXPO
            )
        )
        self._facing_angle_req = (
            requests.FieldCentricFacingAngle()
            .with_deadband(0.0)
            .with_rotational_deadband(0.0)
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
            .with_steer_request_type(
                swerve.SwerveModule.SteerRequestType.MOTION_MAGIC_EXPO
            )
        )
        # Field-absolute version (no operator-perspective rotation) — used by
        # the shooter controller whose target_angle is already in field coords.
        self._facing_angle_field_req = (
            requests.FieldCentricFacingAngle()
            .with_deadband(0.0)
            .with_rotational_deadband(0.0)
            .with_forward_perspective(requests.ForwardPerspectiveValue.BLUE_ALLIANCE)
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
            .with_steer_request_type(
                swerve.SwerveModule.SteerRequestType.MOTION_MAGIC_EXPO
            )
        )
        self._brake_req = requests.SwerveDriveBrake()
        self._sysid_translation_req = requests.SysIdSwerveTranslation()
        self._sysid_rotation_req = requests.SysIdSwerveRotation()
        self._apply_speeds_req = (
            requests.ApplyRobotSpeeds()
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
            .with_steer_request_type(
                swerve.SwerveModule.SteerRequestType.MOTION_MAGIC_EXPO
            )
        )

        self.still_states = self.swerve_module_states
        SmartDashboard.putData("Swerve Drive", self)

        self.period = 0.02
        self.desired_pose = Pose2d()
        self.starting_pose = None  # only used in sim

        self.pigeon_alert = Alert(
            "Pigeon heading has been reset.", AlertType.INFO, timeout=3.0
        )

        self._last_adv_scope_time = 0.0
        self._last_telem_time = 0.0

        # Register telemetry callback (called from the odometry thread)
        self._drivetrain.register_telemetry(self._telemetry_callback)

    def _telemetry_callback(
        self, state: swerve.SwerveDrivetrain.SwerveDriveState
    ) -> None:
        """Called from the odometry thread — keep it cheap."""
        pass  # state is cached internally by SwerveDrivetrain.get_state()

    def on_enable(self):
        # PID controllers for autonomous pose tracking
        self.x_controller = self.translation_profile.create_wpi_pid_controller()
        self.y_controller = self.translation_profile.create_wpi_pid_controller()
        self.theta_controller = (
            self.rotation_profile.create_wpi_profiled_pid_controller_radians()
        )
        self.holonomic_controller = HolonomicDriveController(
            self.x_controller, self.y_controller, self.theta_controller
        )
        self.theta_controller.enableContinuousInput(-math.pi, math.pi)
        self.smart_theta_controller = SmartController(
            "Theta Controller", self.theta_controller.calculate, True
        )

        # Sync facing-angle request PID from the rotation profile
        rp = self.rotation_profile.gains
        heading_kp = rp.get("kP", 1.0)
        heading_ki = rp.get("kI", 0.0)
        heading_kd = rp.get("kD", 0.0)
        max_rot = rp.get("kMaxV", 0)
        for req in (self._facing_angle_req, self._facing_angle_field_req):
            req.with_heading_pid(heading_kp, heading_ki, heading_kd)
            if max_rot > 0:
                req.with_max_abs_rotational_rate(max_rot)

        # Apply steer & drive gains from SmartProfiles to all modules
        if self.tuning_enabled:
            self._apply_motor_gains()

        # Set operator perspective based on alliance colour
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            self._drivetrain.set_operator_perspective_forward(
                Rotation2d.fromDegrees(180)
            )
        else:
            self._drivetrain.set_operator_perspective_forward(Rotation2d())

    def initSendable(self, builder: SendableBuilder) -> None:
        builder.setSmartDashboardType("SwerveDrive")
        builder.addDoubleProperty(
            "Robot Angle",
            lambda: self.get_estimated_pose().rotation().degrees(),
            lambda _: None,
        )
        for i, label in enumerate(
            ("Front Left", "Front Right", "Back Left", "Back Right")
        ):
            _i = i  # capture for closures

            def _vel(idx=_i):
                return self.swerve_module_states[idx].speed * 5

            def _ang(idx=_i):
                return self.swerve_module_states[idx].angle.degrees()

            builder.addDoubleProperty(f"{label} Velocity", _vel, lambda _: None)
            builder.addDoubleProperty(f"{label} Angle", _ang, lambda _: None)

    """
    INFORMATIONAL METHODS
    """

    def get_estimated_pose(self) -> Pose2d:
        return self._cached_pose

    def get_velocity(self) -> ChassisSpeeds:
        return self.chassis_speeds

    def get_module_states(
        self,
    ) -> tuple[
        SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState
    ]:
        return self.swerve_module_states

    @fms_feedback
    def get_distance_from_desired_pose(self) -> units.meters:
        return self.desired_pose.translation().distance(
            self.get_estimated_pose().translation()
        )

    def get_distance_from_pose(self, pose: Pose2d) -> units.meters:
        return pose.translation().distance(self.get_estimated_pose().translation())

    def at_angle(self) -> bool:
        angle_error = abs(
            (
                self.desired_pose.rotation() - self.get_estimated_pose().rotation()
            ).degrees()
        )
        return angle_error < 3.0

    def _apply_motor_gains(self) -> None:
        """Read the latest SmartProfile gains and push them to every module."""
        # Build Slot0 for steer motors
        sg = self.steer_profile.gains
        steer_slot0 = (
            configs.Slot0Configs()
            .with_k_p(sg["kP"])
            .with_k_i(sg["kI"])
            .with_k_d(sg["kD"])
            .with_k_s(sg["kS"])
            .with_k_v(sg["kV"])
            .with_k_a(sg["kA"])
            .with_static_feedforward_sign(
                StaticFeedforwardSignValue.USE_CLOSED_LOOP_SIGN
            )
        )

        # Build Slot0 for drive motors
        dg = self.drive_profile.gains
        drive_slot0 = (
            configs.Slot0Configs()
            .with_k_p(dg["kP"])
            .with_k_i(dg["kI"])
            .with_k_d(dg["kD"])
            .with_k_s(dg["kS"])
            .with_k_v(dg["kV"])
            .with_k_a(dg["kA"])
        )

        for i in range(4):
            mod = self._drivetrain.get_module(i)
            mod.steer_motor.configurator.apply(steer_slot0)
            mod.drive_motor.configurator.apply(drive_slot0)

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
        if field_relative:
            self._pending_request = (
                self._field_centric_req.with_velocity_x(translationX)
                .with_velocity_y(translationY)
                .with_rotational_rate(rotationX)
            )
        else:
            self._pending_request = (
                self._robot_centric_req.with_velocity_x(translationX)
                .with_velocity_y(translationY)
                .with_rotational_rate(rotationX)
            )

    def drive_point(
        self,
        vx: units.meters_per_second,
        vy: units.meters_per_second,
        angle: units.radians,
    ):
        """Drive while pointing the robot at a field-absolute angle.
        Uses BLUE_ALLIANCE perspective (for shooter / field-relative targets)."""
        self._pending_request = (
            self._facing_angle_field_req.with_velocity_x(vx)
            .with_velocity_y(vy)
            .with_target_direction(Rotation2d(angle))
        )

    def drive_point_joy(
        self,
        vx: units.meters_per_second,
        vy: units.meters_per_second,
        joy_x: float,
        joy_y: float,
    ):
        """Drive while pointing the robot in the direction of the right
        joystick.  Uses OPERATOR_PERSPECTIVE so 'push forward' = face away
        from the driver."""
        angle = math.atan2(joy_y, joy_x) - math.radians(90)
        self._pending_request = (
            self._facing_angle_req.with_velocity_x(vx)
            .with_velocity_y(vy)
            .with_target_direction(Rotation2d(angle))
        )

    def sysid_drive(self, volts: float, rot: float = 0.0) -> None:
        self._pending_request = self._sysid_translation_req.with_volts(volts)

    def sysid_rot(self, volts: float, rot: float = 0.0) -> None:
        self._pending_request = self._sysid_rotation_req.with_rotational_rate(volts)

    def set_desired_pose(self, pose: Pose2d):
        self.desired_pose = pose
        if self.get_distance_from_pose(pose) > 0.02:
            speeds = self.holonomic_controller.calculate(
                self.get_estimated_pose(),
                pose,
                0.0,
                pose.rotation(),
            )
            self._pending_request = self._apply_speeds_req.with_speeds(speeds)

    def reset_gyro(self) -> None:
        self._drivetrain.seed_field_centric()
        self.pigeon_alert.enable()

    def addVisionPoseEstimate(self, pose: Pose2d, timestamp: units.seconds):
        self._drivetrain.add_vision_measurement(
            pose, utils.fpga_to_current_time(timestamp)
        )

    def follow_trajectory(self, sample: SwerveSample):
        pose = self.get_estimated_pose()
        # Compute field-relative speeds: feedforward from trajectory + PID feedback
        field_speeds = ChassisSpeeds(
            sample.vx + self.x_controller.calculate(pose.X(), sample.x),
            sample.vy + self.y_controller.calculate(pose.Y(), sample.y),
            sample.omega
            + self.theta_controller.calculate(
                pose.rotation().radians(), sample.heading
            ),
        )
        # Convert field-relative to robot-relative for ApplyRobotSpeeds
        robot_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            field_speeds.vx, field_speeds.vy, field_speeds.omega, pose.rotation()
        )
        self._pending_request = self._apply_speeds_req.with_speeds(robot_speeds)

    def point_towards_joy(
        self,
        rightX: float,
        rightY: float,
        translationX: units.meters_per_second = 0.0,
        translationY: units.meters_per_second = 0.0,
        field_relative: bool = True,
        period: units.seconds = 0.02,
    ):
        """Point the robot in the direction the right joystick is pushed.
        Uses operator-perspective so "push forward" = face away from driver."""
        moved = abs(rightX) > 0.1 or abs(rightY) > 0.1
        if not moved:
            self.drive(translationX, translationY, 0.0, field_relative, period)
            return
        angle = math.atan2(rightY, rightX) - math.radians(90)
        self._pending_request = (
            self._facing_angle_req.with_velocity_x(translationX)
            .with_velocity_y(translationY)
            .with_target_direction(Rotation2d(angle))
        )

    def point_towards(
        self,
        angle: units.radians,
        translationX: units.meters_per_second = 0.0,
        translationY: units.meters_per_second = 0.0,
        field_relative: bool = True,
        period: units.seconds = 0.02,
    ):
        """Point the robot at a field-absolute angle while translating.
        Used by the shooter controller."""
        self._pending_request = (
            self._facing_angle_field_req.with_velocity_x(translationX)
            .with_velocity_y(translationY)
            .with_target_direction(Rotation2d(angle))
        )

    def driveRobotRelative(self, speeds: ChassisSpeeds):
        return self.drive(speeds.vx, speeds.vy, speeds.omega, False, 0.02)

    def set_starting_pose(self, pose: Pose2d):
        """ONLY USE IN SIM!"""
        self.starting_pose = pose
        if pose is not None:
            self._drivetrain.reset_pose(pose)

    def resetPose(self):
        self._drivetrain.reset_pose(Pose2d())

    """
    TELEMETRY
    """

    def sendAdvantageScopeData(self, drive_state=None):
        if not self.adv_scope_enabled:
            return
        now = Timer.getFPGATimestamp()
        if now - self._last_adv_scope_time < self.adv_scope_period:
            return
        self._last_adv_scope_time = now

        swerve_setpoints = []
        for state in self.swerve_module_states:
            swerve_setpoints += [state.angle.degrees(), state.speed]
        SmartDashboard.putNumberArray("Swerve Setpoints", swerve_setpoints)

        if drive_state and drive_state.module_states:
            swerve_measurements = []
            for ms in drive_state.module_states:
                swerve_measurements += [ms.angle.degrees(), ms.speed]
            SmartDashboard.putNumberArray("Swerve Measurements", swerve_measurements)

    def log(self, sys_id_routine: SysIdRoutineLog) -> None:
        """SysId logging: record voltage, position, and velocity for each drive motor."""
        for i, name in enumerate(("fl", "fr", "rl", "rr")):
            mod = self._drivetrain.get_module(i)
            drive_motor = mod.drive_motor
            sys_id_routine.motor(f"swerve/drive/{name}").voltage(
                drive_motor.get_motor_voltage().value
            ).position(drive_motor.get_position().value).velocity(
                drive_motor.get_velocity().value
            )

    def doTelemetry(self):
        """Placeholder — Phoenix 6 odometry thread handles telemetry."""
        pass

    """
    EXECUTE
    """

    def execute(self) -> None:
        # Refresh cached state — single get_state() per cycle.
        # All getters (get_estimated_pose, get_velocity, get_module_states)
        # return these cached values to avoid repeated C++→Python copies.
        drive_state = self._drivetrain.get_state()
        if drive_state:
            self._cached_pose = drive_state.pose if drive_state.pose else Pose2d()
            self.chassis_speeds = (
                drive_state.speeds if drive_state.speeds else ChassisSpeeds()
            )
            if drive_state.module_states:
                self.swerve_module_states = tuple(drive_state.module_states)

        self.sendAdvantageScopeData(drive_state)

        # Apply the pending request (set by a control method on the previous
        # cycle, since drive_control runs after swerve_drive
        # If nothing was requested, brake.
        if self._pending_request is not None:
            self._drivetrain.set_control(self._pending_request)
        else:
            self._drivetrain.set_control(self._brake_req)

        # Clear so that if nothing calls a control method next cycle we brake.
        self._pending_request = None
