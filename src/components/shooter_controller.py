import math
from magicbot import will_reset_to, feedback, StateMachine, state, timed_state
from components.shooter import Shooter
from components.swerve_drive import SwerveDrive
from components.drive_control import DriveControl
from game import get_hub_pos
from wpilib import DriverStation
from lemonlib.smart import SmartPreference
from wpimath.kinematics import ChassisSpeeds

class ShooterController(StateMachine):
    drive_control: DriveControl

    shooter: Shooter
    swerve_drive: SwerveDrive

    projectile_speed: float = 20.0 # m/s, needs tuning

    at_speed: bool = will_reset_to(False)
    shooting: bool = will_reset_to(False)

    idle_speed_scalar = SmartPreference(0.8)

    def setup(self):
        # Meters
        self.distance_lookup = [1.0, 2.0, 3.0, 4.0, 5.0]  # TODO Tune these values

        # RPS
        self.speed_lookup = [22.0, 33.0, 44.0, 55.0, 66.0]  # TODO Tune these values

        self.drive_scalar = 1.0
        self.target_rps = 0.0
        self.speed_tolerance = 0.05  # 5% tolerance
        self.target_angle = 0.0

    def request_shoot(self):
        self.shooting = True

    def _update_target(self):
        # Get target angle from velocity
        chassis_speeds = self.swerve_drive.get_velocity()
        vx = chassis_speeds.vx
        vy = chassis_speeds.vy
        lead_x = vx / self.projectile_speed
        lead_y = vy / self.projectile_speed

        self.target_angle += math.atan2(lead_y, lead_x)
        
        # Determine distance to hub
        robot_pos = self.swerve_drive.get_estimated_pose().translation()
        is_red = DriverStation.getAlliance() == DriverStation.Alliance.kRed
        hub_pos = get_hub_pos(is_red)
        distance = robot_pos.distance(hub_pos)
        self.target_angle = math.atan2(hub_pos.y - robot_pos.y, hub_pos.x - robot_pos.x)
        
        # Linear interpolation without numpy
        self.target_rps = self._linear_interp(distance, self.distance_lookup, self.speed_lookup)
    
    def _linear_interp(self, x, xp, fp):
        """Fast linear interpolation without numpy."""
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

    @state(first=True)
    def idle(self):
        self._update_target()

        self.shooter.set_velocity(self.target_rps * self.idle_speed_scalar)
        if self.shooting:
            self.next_state("align")

    @state
    def align(self):
        self._update_target()

        self.shooter.set_velocity(self.target_rps)
        self.drive_control.point_to(self.target_angle)
        if self.shooting:
            self.next_state("spin_up")
        else:
            self.next_state("idle")

    @state
    def spin_up(self):
        self._update_target()

        self.shooter.set_velocity(self.target_rps)
        self.drive_control.point_to(self.target_angle)

        tolerance = self.speed_tolerance * self.target_rps  # 5% tolerance
        if not self.shooting:
            self.next_state("idle")
        elif abs(self.shooter.get_velocity() - self.target_rps) <= tolerance:
            self.at_speed = True
            self.next_state("shoot")
        else:
            self.at_speed = False

    @state
    def shoot(self):
        self._update_target()

        # aim while still driving
        self.drive_control.point_to(self.target_angle)
        self.shooter.set_velocity(self.target_rps)

        # TODO: Activate indexer here

        if not self.shooting:
            self.next_state("idle")


    @state
    def shoot(self):
        self._update_target()

        self.drive_control.point_to(self.target_angle)
        self.shooter.set_velocity(self.target_rps)
        # TODO: Activate indexer here

        if not self.shooting:
            self.next_state("idle")