import wpilib
from magicbot import MagicRobot
from wpimath.geometry import Rotation2d
from wpimath import units
from phoenix6.hardware import CANcoder, TalonFX
from phoenix6 import CANBus

from components.swerve_wheel import SwerveWheel
from components.swerve_tuner import SwerveTuner
from components.analytical_tuner import AnalyticalTuner
from components.trial_error_tuner import TrialErrorTuner
from lemonlib.smart import SmartProfile


class MyRobot(MagicRobot):

    tuner: SwerveTuner

    analytical_tuner: AnalyticalTuner
    trial_tuner: TrialErrorTuner
    
    front_left: SwerveWheel

    def createObjects(self):
        """Initialize all robot objects here"""
        # Detect if we're in simulation
        self.is_simulation = wpilib.RobotBase.isSimulation()

        # Create joystick for manual control
        self.joystick = wpilib.XboxController(0)

        # Flag to track if tuning is active
        self.tuning_active = False
        self.tuning_enabled = True

        self.canicore_canbus = CANBus("can0")
        # Front Left Module - CAN IDs: Speed=1, Direction=2, CANcoder=3
        self.front_left_speed_motor = TalonFX(11, self.canicore_canbus)
        self.front_left_direction_motor = TalonFX(12, self.canicore_canbus)
        self.front_left_cancoder = CANcoder(13, self.canicore_canbus)

        # Configure module parameters (adjust these for your robot)
        self.wheel_radius = 0.05
        self.drive_gear_ratio = 1 / ((14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0))
        self.direction_gear_ratio = 150.0 / 7.0
        self.direction_amps: units.amperes = 40.0
        self.speed_amps: units.amperes = 60.0

        self.direction_profile = SmartProfile(
            "direction",
            {
                "kP": 0.0,
                "kI": 0.0,
                "kD": 0.0,
                "kS": 0.0,
                "kV": 0.0,
            },
            True,
        )
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
            False,
        )
        self.previous_state = None
        self.state = None

    def teleopPeriodic(self):
        # self.tuner.engage()
        """Called periodically during teleop mode"""
        self.tuner.engage()

        if self.joystick.getBackButtonPressed():
            print("Starting Swerve Tuning...")
            self.tuner.start_tuning()


if __name__ == "__main__":
    wpilib.run(MyRobot)
