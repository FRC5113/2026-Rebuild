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

    shooter_height = 0.5  # TUNE, RANDOM NUM
    hub_height = 2  # TUNE, RANDOM NUM
    gravity = 9.81

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

    # TODO Detect when to stop
    def stop_shoot(self):
        self.shooting = False

    def _update_target(self):        
        robot_pos = self.swerve_drive.get_estimated_pose().translation()
        is_red = DriverStation.getAlliance() == DriverStation.Alliance.kRed
        hub_pos = get_hub_pos(is_red)
        
        chassis_speeds = self.swerve_drive.get_velocity()
        vx = chassis_speeds.vx
        vy = chassis_speeds.vy
        
        height_diff = self.hub_height - self.shooter_height
        
        predicted_x = hub_pos.x
        predicted_y = hub_pos.y
        
        for _ in range(3):
            dx = predicted_x - robot_pos.x
            dy = predicted_y - robot_pos.y
            
            # magnitude of vector on ground from robot to hub
            horizontal_distance = math.sqrt(dx * dx + dy * dy)

            time_to_target = self._calculate_flight_time(horizontal_distance, height_diff)
            
            predicted_x = hub_pos.x + vx * time_to_target
            predicted_y = hub_pos.y + vy * time_to_target
        
        self.target_angle = math.atan2(predicted_y - robot_pos.y, predicted_x - robot_pos.x)
        
        # speed lookup with original distance (predicted only used for angle)
        original_distance = robot_pos.distance(hub_pos)
        self.target_rps = self._linear_interp(original_distance, self.distance_lookup, self.speed_lookup)

    def _calculate_flight_time(self, horizontal_distance, height_diff):
        """
        Calculate time of flight for a projectile, given horizontal distance and height difference.

        Uses approximate tangent for speed.
        """
        v0 = self.projectile_speed
        g = self.gravity
        d = horizontal_distance
        h = height_diff
        
        # From projectile motion, solving for time:
        # The equation is: h = d*tan(θ) - (g*d²)/(2*v0²*cos²(θ))
        # where θ is the launch angle
        # Time is: t = d / (v0 * cos(θ))
        
        # Rearranging the trajectory equation gives us:
        # (g*d²/(2*v0²)) * sec²(θ) - d*tan(θ) + h = 0
        # Using sec²(θ) = 1 + tan²(θ), this becomes a quadratic in tan(θ):
        # (g*d²/(2*v0²)) * tan²(θ) - d*tan(θ) + (g*d²/(2*v0²) + h) = 0
        
        a_coeff = g * d * d / (2 * v0 * v0)
        b_coeff = -d
        c_coeff = a_coeff + h
        
        discriminant = b_coeff * b_coeff - 4 * a_coeff * c_coeff
        
        if discriminant < 0:
            # Target is unreachable with current velocity, best guess returned
            return math.sqrt(d*d + h*h) / v0
        
        sqrt_disc = math.sqrt(discriminant)
        
        # Two solutions for tan(θ)
        tan_theta_1 = (-b_coeff - sqrt_disc) / (2 * a_coeff)
        tan_theta_2 = (-b_coeff + sqrt_disc) / (2 * a_coeff)
        
        # Choose the smaller angle (flatter trajectory) - smaller tan(θ)
        # This is typically tan_theta_1
        tan_theta = tan_theta_1 if abs(tan_theta_1) < abs(tan_theta_2) else tan_theta_2
        
        # Calculate time from: t = d / (v0 * cos(θ))
        # cos(θ) = 1 / sqrt(1 + tan²(θ))
        cos_theta = 1.0 / math.sqrt(1 + tan_theta * tan_theta)
        
        time = d / (v0 * cos_theta)
        
        return time if time > 0 else d / v0
    
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
