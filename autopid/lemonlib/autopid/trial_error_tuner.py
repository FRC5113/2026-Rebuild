"""
Generic trial-and-error tuning component using empirical methods.
Incrementally tunes kP, kI, and kD gains through observation.
"""

from typing import List, Optional

import wpilib
from wpilib import RobotBase, RuntimeType

from ..smart import SmartNT
from .motor_interface import ControlMode, MotorInterface
from .tuning_data import (
    ControlType,
    FeedforwardGains,
    GravityType,
    MechanismTuningResults,
    MotorGains,
)


class TrialErrorTuner:
    """
    Component that performs trial-and-error (empirical) tuning.
    Incrementally adjusts gains based on system response.
    Works with any mechanism type.
    """

    def setup(self):
        self.nt = SmartNT("Trial Error Tuner")

        # Tuning increments (will be adjusted based on mechanism)
        self.kP_increment = 0.5
        self.kI_increment = 0.01
        self.kD_increment = 0.01

        # State tracking
        self.is_running = False
        self.is_done = False
        self.current_motor: Optional[MotorInterface] = None
        self.results: Optional[MechanismTuningResults] = None

        # Current gain values being tested
        self.kP_trial = 0.0
        self.kI_trial = 0.0
        self.kD_trial = 0.0

        # Feedforward gains used during tuning
        self.feedforward_gains = FeedforwardGains()

        # Data collection
        self.position_samples: List[float] = []
        self.velocity_samples: List[float] = []
        self.time_samples: List[float] = []

        # Timing
        self.timer = wpilib.Timer()

        # Test state
        self.current_test = "idle"  # idle, kP, kI, kD

        # Tuning tracking
        self.velocity_setpoint = 2.0  # units/s for velocity tuning
        self.position_setpoint = 0.0  # For position tuning

    """
    CONTROL METHODS
    """

    def start(
        self,
        motor: MotorInterface,
        control_type: ControlType,
        gravity_type: GravityType,
        name: str,
        initial_gains: Optional[MotorGains] = None,
        feedforward_gains: Optional[FeedforwardGains] = None,
    ) -> None:
        """
        Start trial-and-error tuning for a mechanism.

        Args:
            motor: MotorInterface wrapping the motor controller
            control_type: Position or Velocity control
            gravity_type: Type of gravity compensation needed
            name: Human-readable name for the mechanism
            initial_gains: Optional analytical gains to use as starting point
            feedforward_gains: Optional feedforward gains to apply while tuning PID
        """
        self.current_motor = motor
        self.is_running = True
        self.is_done = False

        motor_id = motor.get_motor_id()

        # Get or create results
        if self.results and self.results.mechanism_id == motor_id:
            # Continue with existing results
            pass
        else:
            self.results = MechanismTuningResults(
                mechanism_id=motor_id,
                mechanism_name=name,
                control_type=control_type,
                gravity_type=gravity_type,
            )

        # Store feedforward gains for use during tuning
        self.feedforward_gains = feedforward_gains or FeedforwardGains()
        if self.results:
            self.results.feedforward_gains = self.feedforward_gains

        # Initialize trial gains from analytical if provided
        if initial_gains:
            self.kP_trial = initial_gains.kP * 0.3
            self.kI_trial = 0.0  # Always start kI from zero
            self.kD_trial = 0.0  # Always start kD from zero
        else:
            self.kP_trial = 0.0
            self.kI_trial = 0.0
            self.kD_trial = 0.0

        # Start with kP
        self.current_test = "kP"
        self.timer.restart()
        self._reset_data()

        # Apply initial zero PID gains with provided feedforward
        self.current_motor.apply_gains(
            kS=self.feedforward_gains.kS,
            kV=self.feedforward_gains.kV,
            kA=self.feedforward_gains.kA,
            kP=0.0,
            kI=0.0,
            kD=0.0,
            kG=self.feedforward_gains.kG,
        )

        # self.nt.put(f"{self.results.mechanism_name}/Trial kS", 0.0)
        # self.nt.put(f"{self.results.mechanism_name}/Trial kV", 0.0)
        # self.nt.put(f"{self.results.mechanism_name}/Trial kA", 0.0)
        # self.nt.put(f"{self.results.mechanism_name}/Trial kP", 0.0)
        # self.nt.put(f"{self.results.mechanism_name}/Trial kI", 0.0)
        # self.nt.put(f"{self.results.mechanism_name}/Trial kD", 0.0)
        # self.nt.put(f"{self.results.mechanism_name}/Trial kG", 0.0)
        # self.nt.put(f"{self.results.mechanism_name}/kV Error", 0.0)

        print(f"Trial-and-error tuning started for {name}")

    def stop(self) -> None:
        """Stop the current tuning process"""
        self.is_running = False
        if self.current_motor:
            self.current_motor.set_control(ControlMode.BRAKE)

    def is_complete(self) -> bool:
        """Check if tuning is complete"""
        return self.is_done

    def get_results(self) -> Optional[MechanismTuningResults]:
        """Get the tuning results"""
        return self.results

    def get_trial_gains(self) -> Optional[MotorGains]:
        """Get just the trial gains"""
        if self.results:
            return self.results.trial_gains
        return None

    """
    HELPER METHODS
    """

    def _reset_data(self):
        """Clear data collection arrays"""
        self.position_samples.clear()
        self.velocity_samples.clear()
        self.time_samples.clear()
        self.timer.restart()

    def _collect_data(self, elapsed: float):
        """Collect position and velocity data"""
        if self.current_motor:
            position = self.current_motor.get_position()
            velocity = self.current_motor.get_velocity()

            self.position_samples.append(position)
            self.velocity_samples.append(velocity)
            self.time_samples.append(elapsed)

    def _detect_oscillation(self) -> bool:
        """Detect position oscillation"""
        if len(self.position_samples) < 30:
            return False

        recent_samples = self.position_samples[-30:]
        target = sum(recent_samples) / len(recent_samples)
        crossings = 0

        for i in range(1, len(recent_samples)):
            if (recent_samples[i - 1] < target and recent_samples[i] > target) or (
                recent_samples[i - 1] > target and recent_samples[i] < target
            ):
                crossings += 1

        return crossings >= 4

    def _detect_jitter(self) -> bool:
        """Detect velocity jitter"""
        if len(self.velocity_samples) < 30:
            return False

        recent_velocities = self.velocity_samples[-30:]
        mean_vel = sum(recent_velocities) / len(recent_velocities)
        variance = sum((v - mean_vel) ** 2 for v in recent_velocities) / len(
            recent_velocities
        )

        return variance > 0.5

    """
    TEST METHODS
    """

    def _tune_kP(self, elapsed: float):
        """Tune kP by increasing until oscillation"""
        self._collect_data(elapsed)

        # Apply current gains
        self.current_motor.apply_gains(
            kP=self.kP_trial,
            kI=0.0,
            kD=0.0,
            kS=self.feedforward_gains.kS,
            kV=self.feedforward_gains.kV,
            kA=self.feedforward_gains.kA,
            kG=self.feedforward_gains.kG,
        )

        # Use appropriate control mode
        if self.results.control_type == ControlType.POSITION:
            self.current_motor.set_control(ControlMode.POSITION, self.position_setpoint)
        else:
            self.current_motor.set_control(ControlMode.VELOCITY, self.velocity_setpoint)

        self.nt.put(f"{self.results.mechanism_name}/Trial kP", self.kP_trial)

        # Check for oscillation after settling
        if elapsed > 2.0 and len(self.position_samples) > 30:
            if self._detect_oscillation():
                self.kP_trial = max(0.0, self.kP_trial - self.kP_increment)
                self.results.trial_gains.kP = self.kP_trial
                self.nt.put(f"{self.results.mechanism_name}/Trial kP", self.kP_trial)
                print(f"{self.results.mechanism_name} - Trial kP: {self.kP_trial:.4f}")

                # For velocity control, may want kI
                if self.results.control_type == ControlType.VELOCITY:
                    self.current_test = "kI"
                else:
                    self.current_test = "kD"
                self._reset_data()
                return

        # Increment every 1.0 seconds
        if elapsed > 2.0 and int(elapsed) != int(elapsed - 0.02):
            self.kP_trial += self.kP_increment

        # Timeout or simulation
        if elapsed > 120.0 or (
            RobotBase.getRuntimeType() == RuntimeType.kSimulation and elapsed > 2.0
        ):
            if RobotBase.getRuntimeType() == RuntimeType.kSimulation:
                self.kP_trial = 10.0
            self.results.trial_gains.kP = self.kP_trial
            print(
                f"{self.results.mechanism_name} - Trial kP (timeout/sim): {self.kP_trial:.4f}"
            )
            if self.results.control_type == ControlType.VELOCITY:
                self.current_test = "kI"
            else:
                self.current_test = "kD"
            self._reset_data()

    def _tune_kI(self, elapsed: float):
        """Tune kI for velocity control (helps with steady-state error)"""
        self._collect_data(elapsed)

        # Apply current gains
        self.current_motor.apply_gains(
            kP=self.kP_trial,
            kI=self.kI_trial,
            kD=0.0,
            kS=self.feedforward_gains.kS,
            kV=self.feedforward_gains.kV,
            kA=self.feedforward_gains.kA,
            kG=self.feedforward_gains.kG,
        )
        self.current_motor.set_control(ControlMode.VELOCITY, self.velocity_setpoint)
        self.nt.put(f"{self.results.mechanism_name}/Trial kI", self.kI_trial)

        # Check for instability or oscillation
        if elapsed > 2.0 and len(self.velocity_samples) > 30:
            if self._detect_jitter():
                self.kI_trial = max(0.0, self.kI_trial - self.kI_increment)
                self.results.trial_gains.kI = self.kI_trial
                self.nt.put(f"{self.results.mechanism_name}/Trial kI", self.kI_trial)
                print(f"{self.results.mechanism_name} - Trial kI: {self.kI_trial:.4f}")

                # Done!
                self.is_done = True
                self.is_running = False
                self.current_test = "idle"
                return

        # Increment every 1.0 seconds
        if elapsed > 2.0 and int(elapsed) != int(elapsed - 0.02):
            self.kI_trial += self.kI_increment

        # Timeout or simulation
        if elapsed > 120.0 or (
            RobotBase.getRuntimeType() == RuntimeType.kSimulation and elapsed > 2.0
        ):
            if RobotBase.getRuntimeType() == RuntimeType.kSimulation:
                self.kI_trial = 0.5
            self.results.trial_gains.kI = self.kI_trial
            print(
                f"{self.results.mechanism_name} - Trial kI (timeout/sim): {self.kI_trial:.4f}"
            )
            self.is_done = True
            self.is_running = False
            self.current_test = "idle"

    def _tune_kD(self, elapsed: float):
        """Tune kD by increasing until jitter (position control)"""
        self._collect_data(elapsed)

        # Apply current gains with full PD
        self.current_motor.apply_gains(
            kP=self.kP_trial,
            kI=0.0,
            kD=self.kD_trial,
            kS=self.feedforward_gains.kS,
            kV=self.feedforward_gains.kV,
            kA=self.feedforward_gains.kA,
            kG=self.feedforward_gains.kG,
        )
        self.current_motor.set_control(ControlMode.POSITION, self.position_setpoint)
        self.nt.put(f"{self.results.mechanism_name}/Trial kD", self.kD_trial)

        # Check for jitter after settling
        if elapsed > 2.0 and len(self.velocity_samples) > 30:
            if self._detect_jitter():
                self.kD_trial = max(0.0, self.kD_trial - self.kD_increment)
                self.results.trial_gains.kD = self.kD_trial
                self.nt.put(f"{self.results.mechanism_name}/Trial kD", self.kD_trial)
                print(f"{self.results.mechanism_name} - Trial kD: {self.kD_trial:.4f}")

                # Done!
                self.is_done = True
                self.is_running = False
                self.current_test = "idle"
                return

        # Increment every 1.0 seconds
        if elapsed > 2.0 and int(elapsed) != int(elapsed - 0.02):
            self.kD_trial += self.kD_increment

        # Timeout or simulation
        if elapsed > 120.0 or (
            RobotBase.getRuntimeType() == RuntimeType.kSimulation and elapsed > 2.0
        ):
            if RobotBase.getRuntimeType() == RuntimeType.kSimulation:
                self.kD_trial = 0.1
            self.results.trial_gains.kD = self.kD_trial
            print(
                f"{self.results.mechanism_name} - Trial kD (timeout/sim): {self.kD_trial:.4f}"
            )
            self.is_done = True
            self.is_running = False
            self.current_test = "idle"

    """
    EXECUTE METHOD
    """

    def execute(self):
        """Called every robot loop - runs the current test"""
        if not self.is_running or self.current_motor is None:
            return

        elapsed = self.timer.get()

        # Run the current test
        if self.current_test == "kP":
            self._tune_kP(elapsed)
        elif self.current_test == "kI":
            self._tune_kI(elapsed)
        elif self.current_test == "kD":
            self._tune_kD(elapsed)

        self.current_motor.periodic()
