import math

from magicbot import StateMachine, state, will_reset_to
from wpilib import DriverStation

from components.drive_control import DriveControl
from components.shooter import Shooter
from components.swerve_drive import SwerveDrive
from game import get_hub_pos
from lemonlib import fms_feedback


class ShooterController(StateMachine):
    drive_control: DriveControl

    shooter: Shooter
    swerve_drive: SwerveDrive

    at_speed: bool = will_reset_to(False)
    shooting: bool = will_reset_to(False)

    idle_speed_scalar = 0.8
    kicker_duty = 8  # Volts
    angle_tolerance = 0.035  # ~2 degrees in radians
    speed_tolerance = 0.05  # 5% relative tolerance

    def setup(self):
        # Meters
        self.distance_lookup = [4.0]  # TODO Tune these values

        # RPS
        self.speed_lookup = [53.0]  # TODO Tune these values

        # Seconds — measured flight times at each distance
        self.time_lookup = [0.45]  # TODO Tune these values

        self.target_rps = 0.0
        self.target_angle = 0.0
        self.distance = 0.0

    def request_shoot(self):
        self.shooting = True

    def _update_target(self):
        robot_pos = self.swerve_drive.get_estimated_pose().translation()
        is_red = DriverStation.getAlliance() == DriverStation.Alliance.kRed
        hub_pos = get_hub_pos(is_red)

        chassis_speeds = self.swerve_drive.get_velocity()
        vx = chassis_speeds.vx
        vy = chassis_speeds.vy

        # Iteratively solve for the aim point compensating for robot velocity.
        # The projectile inherits the robot's velocity, so we subtract it
        # from the hub position to find the correct aim point.
        predicted_x = hub_pos.x
        predicted_y = hub_pos.y

        for _ in range(3):
            dx = predicted_x - robot_pos.x
            dy = predicted_y - robot_pos.y

            horizontal_distance = math.hypot(dx, dy)
            time_to_target = self._linear_interp(
                horizontal_distance, self.distance_lookup, self.time_lookup
            )

            predicted_x = hub_pos.x - vx * time_to_target
            predicted_y = hub_pos.y - vy * time_to_target

        self.target_angle = math.atan2(
            predicted_y - robot_pos.y, predicted_x - robot_pos.x
        )

        # Use the compensated distance (to the predicted aim point, not the raw hub)
        dx = predicted_x - robot_pos.x
        dy = predicted_y - robot_pos.y
        self.distance = math.hypot(dx, dy)
        self.target_rps = self._linear_interp(
            self.distance, self.distance_lookup, self.speed_lookup
        )

    def _linear_interp(self, x: float, xp: list[float], fp: list[float]) -> float:
        """Linear interpolation between lookup table points."""

        if x <= xp[0]:
            return fp[0]
        if x >= xp[-1]:
            return fp[-1]

        for i in range(len(xp) - 1):
            if xp[i] <= x <= xp[i + 1]:
                # Linear interpolation formula
                t = (x - xp[i]) / (xp[i + 1] - xp[i])
                return fp[i] + t * (fp[i + 1] - fp[i])

        return fp[-1]

    @fms_feedback
    def get_target_rps(self) -> float:
        return self.target_rps

    @fms_feedback
    def get_distance(self) -> float:
        return self.distance

    @fms_feedback
    def is_at_speed(self) -> bool:
        return self.at_speed

    @state(first=True)
    def idle(self):
        self._update_target()
        self.shooter.set_velocity(self.target_rps * self.idle_speed_scalar)
        if self.shooting:
            self.next_state("spin_up")

    def _is_aimed(self) -> bool:
        """Check if the robot heading is within tolerance of the target angle."""
        heading = self.swerve_drive.get_estimated_pose().rotation().radians()
        error = math.atan2(
            math.sin(self.target_angle - heading),
            math.cos(self.target_angle - heading),
        )
        return abs(error) <= self.angle_tolerance

    @state
    def spin_up(self):
        self._update_target()
        self.shooter.set_velocity(self.target_rps)
        self.drive_control.point_to(self.target_angle)

        if not self.shooting:
            self.next_state("idle")
            return

        tolerance = self.speed_tolerance * self.target_rps
        speed_ready = abs(self.shooter.get_velocity() - self.target_rps) <= tolerance
        aim_ready = self._is_aimed()

        self.at_speed = speed_ready and aim_ready
        if self.at_speed:
            self.next_state("shoot")

    @state
    def shoot(self):
        self._update_target()
        self.drive_control.point_to(self.target_angle)
        self.shooter.set_velocity(self.target_rps)

        # Re-verify conditions — fall back to spin_up if speed or aim drifts
        tolerance = self.speed_tolerance * self.target_rps
        speed_ready = abs(self.shooter.get_velocity() - self.target_rps) <= tolerance
        aim_ready = self._is_aimed()
        self.at_speed = speed_ready and aim_ready

        if not self.shooting:
            self.next_state("idle")
        elif not self.at_speed:
            self.next_state("spin_up")
        else:
            self.shooter.set_kicker(self.kicker_duty)
