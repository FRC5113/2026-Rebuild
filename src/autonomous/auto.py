from enum import Enum
from typing import List
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.units import meters, degrees
from components.swerve_drive import SwerveDrive
from components.shooter import Shooter
from components.intake import Intake
from components.shooter_controller import ShooterController
from magicbot import StateMachine

class StepStatus(Enum):
    RUNNING = 1
    DONE = 2


class AutoStep:
    """Base class for all auto steps"""

    def execute(self, ctx) -> StepStatus:
        raise NotImplementedError


class AutoContext:
    sd: SwerveDrive
    sh: Shooter
    it: Intake
    sc: StateMachine

    def __init__(self, sd: SwerveDrive, sh: Shooter, it: Intake, sc: StateMachine):
        self.sd = sd
        self.sh = sh
        self.it = it
        self.sc = sc


class AutoRunner:
    def __init__(self, steps: List[AutoStep]):
        self.steps = steps
        self.ctx = AutoContext(SwerveDrive(), Shooter(), Intake(), ShooterController()) # Placeholder
        self.index = 0

    def reset(self):
        self.index = 0

    def run(self, ctx: AutoContext):
        if self.index >= len(self.steps):
            return

        status = self.steps[self.index].execute(self.ctx)

        if status == StepStatus.DONE:
            self.index += 1


class ParallelStep(AutoStep):
    """
    Runs multiple steps at the same time.
    Finishes when ALL steps are DONE.
    """

    def __init__(self, *steps: AutoStep):
        self.steps = steps
        # Track which sub-steps have completed so we don't re-execute them
        self._done = [False] * len(steps)

    def execute(self, ctx: AutoContext) -> StepStatus:
        all_done = True
        for i in range(len(self.steps)):
            if self._done[i]:
                continue
            status = self.steps[i].execute(ctx)
            if status == StepStatus.DONE:
                self._done[i] = True
            else:
                all_done = False
        # If any step is not done yet, we're still running
        if not all_done:
            return StepStatus.RUNNING
        return StepStatus.DONE


class SwerveDriveAuto(AutoStep):
    """
    Field-oriented drive: takes absolute field coordinates (x, y in meters, heading in degrees).
    Uses WPILib coordinate system where (0,0) is at field corner from blue alliance perspective.
    """

    POSITION_TOLERANCE = 0.02  # meters

    def __init__(self, x: meters, y: meters, heading: degrees):
        self.x = x
        self.y = y
        self.heading_deg = heading
        self.target_pose = None

    def execute(self, ctx: AutoContext) -> StepStatus:
        # TODO: PID - it should do pid not just naivley drive there and dead stop
        # TODO: Angle tolerance
        if self.target_pose is None:
            self.target_pose = Pose2d(
                self.x, self.y, Rotation2d.fromDegrees(self.heading_deg)
            )
            ctx.sd.set_desired_pose(self.target_pose)

        distance = ctx.sd.get_distance_from_pose(self.target_pose)
        if distance <= self.POSITION_TOLERANCE:
            return StepStatus.DONE
        return StepStatus.RUNNING


class IntakeAuto(AutoStep):
    """Lets you turn on or off the intake with the boolean parameter"""

    def __init__(self, is_on: bool):
        self.is_on = is_on
        self.applied = False

    def execute(self, ctx: AutoContext) -> StepStatus:
        if not self.applied:
            ctx.it.set_voltage(12 if self.is_on else 0)
            self.applied = True
        return StepStatus.DONE


class ShootAuto(AutoStep):
    STATIC_ANGLE = 78  # Not sure this is right

    def __init__(self):
        self.started = False

    def execute(self, ctx: AutoContext) -> StepStatus:
        if not self.started:
            ctx.sc.request_shoot()
            self.started = True

        # Check if shooter is at speed and ready
        if ctx.sc.at_speed:
            return StepStatus.DONE
        return StepStatus.RUNNING


# THESE ARE ALL MADE UP NUMBERS!!!!!!!!!
tempAutoRoutine = AutoRunner(
    [
        SwerveDriveAuto(1.5, 2.11, 45),
        ParallelStep(SwerveDriveAuto(1.5, 4.59, 135), IntakeAuto(True)),
        ParallelStep(SwerveDriveAuto(2.51, 4.59, 45), IntakeAuto(True)),
        IntakeAuto(False),  # Turn off intake before going over bump
        SwerveDriveAuto(2.51, 7.18, -45),
        SwerveDriveAuto(2.51, 8.57, -135),
        ShootAuto(),
    ],
)
