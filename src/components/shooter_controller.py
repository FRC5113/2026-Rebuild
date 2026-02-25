import math

from magicbot import StateMachine, feedback, state, will_reset_to
from wpilib import DriverStation

from components.drive_control import DriveControl
from components.shooter import Shooter
from components.swerve_drive import SwerveDrive
from game import get_hub_pos
from lemonlib.smart import SmartPreference


class ShooterController(StateMachine):
    drive_control: DriveControl

    shooter: Shooter
    swerve_drive: SwerveDrive

    at_speed: bool = will_reset_to(False)
    shooting: bool = will_reset_to(False)

    idle_speed_scalar = SmartPreference(0.8)
    kicker_duty_cycle = SmartPreference(0.67)  # ~8V / 12V
    angle_tolerance = SmartPreference(0.035)  # ~2 degrees in radians

    def setup(self):
        # Meters
        self.distance_lookup = [1.0, 2.0, 3.0, 4.0, 5.0]  # TODO Tune these values

        # RPS
        self.speed_lookup = [22.0, 33.0, 44.0, 55.0, 66.0]  # TODO Tune these values

        # Seconds — measured flight times at each distance
        self.time_lookup = [0.15, 0.25, 0.35, 0.45, 0.55]  # TODO Tune these values

        self.drive_scalar = 1.0
        self.target_rps = 0.0
        self.speed_tolerance = 0.05  # 5% tolerance
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

        self.distance = robot_pos.distance(hub_pos)
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

    @feedback
    def get_target_rps(self) -> float:
        return self.target_rps

    @feedback
    def get_distance(self) -> float:
        return self.distance

    @feedback
    def is_at_speed(self) -> bool:
        return self.at_speed

    @state(first=True)
    def idle(self):
        self._update_target()
        self.shooter.set_velocity(self.target_rps * self.idle_speed_scalar)
        if self.shooting:
            self.next_state("spin_up")

    @state
    def spin_up(self):
        self._update_target()
        self.shooter.set_velocity(self.target_rps)
        self.drive_control.point_to(self.target_angle)

        if not self.shooting:
            self.next_state("idle")
            return

        tolerance = self.speed_tolerance * self.target_rps
        if abs(self.shooter.get_velocity() - self.target_rps) <= tolerance:
            self.at_speed = True
            self.next_state("shoot")
        else:
            self.at_speed = False

    @state
    def shoot(self):
        self._update_target()
        self.drive_control.point_to(self.target_angle)
        self.shooter.set_velocity(self.target_rps)
        self.shooter.set_kicker(self.kicker_duty_cycle)

        if not self.shooting:
            self.next_state("idle")
