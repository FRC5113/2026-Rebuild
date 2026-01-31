"""
Generic motor tuner that can tune any FRC mechanism.
Supports TalonFX and SparkMax motors, position and velocity control,
and various gravity compensation types.
"""

from typing import Optional, Callable
from enum import Enum, auto
import wpilib
from wpimath import units

from .motor_interface import (
    MotorInterface,
    TalonFXInterface,
    WPIMotorControllerInterface,
    ControlMode,
    SparkMaxInterface,
    TalonFXSInterface,
)
from .tuning_data import ControlType, GravityType, MotorGains, MechanismTuningResults,TuningProfile
from .analytical_tuner import AnalyticalTuner
from .trial_error_tuner import TrialErrorTuner


class TuningState(Enum):
    """States for the tuning state machine"""

    IDLE = auto()
    ANALYTICAL_TUNING = auto()
    TRIAL_TUNING = auto()
    COMPLETE = auto()
    FAILED = auto()


class GenericMotorTuner:
    """
    High-level tuner for any FRC mechanism.

    This tuner uses a non-blocking state machine that integrates with the RobotPy periodic loop.

    Basic Usage:
        ```python
        # In robot init:
        self.tuner = None

        # In teleopPeriodic():
        if some_condition and self.tuner is None:
            # Create and start tuning
            self.tuner = GenericMotorTuner(
                motor=my_motor,
                control_type=ControlType.POSITION,
                name="My Mechanism"
            )
            self.tuner.start_tuning(use_analytical=True, use_trial=True)

        # Run tuning state machine each loop
        if self.tuner is not None:
            if not self.tuner.is_complete():
                self.tuner.periodic()
            else:
                gains = self.tuner.get_final_gains()
                print(f"Tuning complete! Gains: {gains}")
                self.tuner = None
        ```

    Using Convenience Functions:
        ```python
        # Swerve module - position control
        self.tuner = tune_swerve_module(motor, "FL Module")
        self.tuner.start_tuning()

        # Flywheel - velocity control
        self.tuner = tune_flywheel(motor, "Shooter Flywheel")
        self.tuner.start_tuning()

        # Elevator - position control with constant gravity
        self.tuner = tune_elevator(motor, "Elevator", height_getter)
        self.tuner.start_tuning()

        # Arm - position control with cosine gravity
        self.tuner = tune_arm(motor, "Arm Pivot", angle_getter)
        self.tuner.start_tuning()

        # Hood - analytical tuning only
        self.tuner = tune_hood(motor, "Hood")
        self.tuner.start_tuning(use_trial=False)
        ```
    """

    def __init__(
        self,
        motor,
        control_type: ControlType,
        name: str,
        tuning_profile = TuningProfile.AUTO,
        motor_interface: Optional[MotorInterface] = None,
        gravity_type: GravityType = GravityType.NONE,
        position_getter: Optional[Callable[[], float]] = None,
        velocity_getter: Optional[Callable[[], units.meters_per_second]] = None,
        conversion_factor: float = 1.0,
    ):
        """
        Initialize the generic motor tuner.

        Args:
            motor: TalonFX or SparkMax motor object
            control_type: POSITION or VELOCITY control
            name: Human-readable name for the mechanism
            gravity_type: Type of gravity compensation (NONE, CONSTANT, COSINE)
            position_getter: Optional custom position getter function
            conversion_factor: For SparkMax, conversion from encoder units to mechanism units
        """
        # Create motor interface
        if motor_interface is None:
            self.motor_interface = self._create_motor_interface(
                motor, position_getter, conversion_factor
            )
        else:
            self.motor_interface = motor_interface

        self.control_type = control_type
        self.gravity_type = gravity_type
        self.name = name
        self.velocity_getter = velocity_getter
        self.tuning_profile = tuning_profile

        # Create tuning components
        self.analytical_tuner = AnalyticalTuner()
        self.trial_tuner = TrialErrorTuner()

        # Initialize components
        self.analytical_tuner.setup()
        self.trial_tuner.setup()

        # Results
        self.results: Optional[MechanismTuningResults] = None

        # State machine
        self.state = TuningState.IDLE
        self.start_time = 0.0
        self.timeout_seconds = 180.0
        self.use_analytical = True
        self.use_trial = True

    def _create_motor_interface(
        self, motor, position_getter, conversion_factor
    ) -> MotorInterface:
        """Create appropriate motor interface based on motor type"""
        # Try to detect motor type
        motor_type_name = type(motor).__name__

        if "TalonFX" in motor_type_name:
            return TalonFXInterface(motor, position_getter)
        if "TalonFXS" in motor_type_name:
            return TalonFXSInterface(motor, position_getter)
        elif "SparkMax" in motor_type_name or "CANSparkMax" in motor_type_name:
            return SparkMaxInterface(motor, conversion_factor, position_getter)
        else:
            # Default to TalonFX interface
            print(
                f"Warning: Unknown motor type {motor_type_name}, defaulting to Wpi MotorController interface"
            )
            return WPIMotorControllerInterface(
                motor, position_getter, velocity_getter=self.velocity_getter
            )

    def start_tuning(
        self,
        use_analytical: bool = True,
        use_trial: bool = True,
        timeout_seconds: float = 180.0,
    ):
        """
        Start the tuning process. Call periodic() repeatedly until is_complete() returns True.

        Args:
            use_analytical: Run analytical tuning phase
            use_trial: Run trial-and-error tuning phase
            timeout_seconds: Maximum time for entire tuning process
        """
        self.use_analytical = use_analytical
        self.use_trial = use_trial
        self.timeout_seconds = timeout_seconds
        self.start_time = wpilib.Timer.getFPGATimestamp()

        print(f"\n{'='*70}")
        print(f"Starting Auto-Tune for: {self.name}")
        print(f"Control Type: {self.control_type.value}")
        print(f"Gravity Type: {self.gravity_type.value}")
        print(
            f"Motor: {self.motor_interface.get_motor_type().value} (ID: {self.motor_interface.get_motor_id()})"
        )
        print(f"{'='*70}\n")

        # Start with analytical tuning if enabled
        if use_analytical:
            self.state = TuningState.ANALYTICAL_TUNING
            print(f"[{self.name}] Starting analytical tuning...")
            self.analytical_tuner.start(
                self.motor_interface, self.control_type, self.gravity_type, self.name, self.tuning_profile
            )
        elif use_trial:
            self._start_trial_tuning()
        else:
            print(f"[{self.name}] Warning: No tuning phases enabled")
            self.state = TuningState.FAILED

    def periodic(self):
        """
        Execute one iteration of the tuning state machine.
        Call this repeatedly from your robot's periodic method until is_complete() returns True.
        """
        # Check timeout
        if wpilib.Timer.getFPGATimestamp() - self.start_time > self.timeout_seconds:
            print(f"[{self.name}] Tuning timed out")
            self._finalize_tuning()
            return

        # Execute current state
        if self.state == TuningState.ANALYTICAL_TUNING:
            self.analytical_tuner.execute()

            if self.analytical_tuner.is_complete():
                analytical_results = self.analytical_tuner.get_results()
                if analytical_results:
                    self.results = analytical_results
                    print(f"[{self.name}] Analytical tuning complete")
                else:
                    print(f"[{self.name}] Analytical tuning failed")
                    # Create empty results
                    self.results = MechanismTuningResults(
                        mechanism_id=self.motor_interface.get_motor_id(),
                        mechanism_name=self.name,
                        control_type=self.control_type,
                        gravity_type=self.gravity_type,
                    )

                # Move to trial tuning if enabled
                if self.use_trial:
                    self._start_trial_tuning()
                else:
                    self._finalize_tuning()

        elif self.state == TuningState.TRIAL_TUNING:
            self.trial_tuner.execute()

            if self.trial_tuner.is_complete():
                trial_results = self.trial_tuner.get_results()
                if trial_results:
                    self.results = trial_results
                    print(f"[{self.name}] Trial-and-error tuning complete")
                else:
                    print(f"[{self.name}] Trial-and-error tuning failed")

                self._finalize_tuning()

    def _start_trial_tuning(self):
        """Transition to trial-and-error tuning phase"""
        self.state = TuningState.TRIAL_TUNING
        print(f"\n[{self.name}] Starting trial-and-error tuning...")

        # Get analytical gains as starting point
        analytical_gains = (
            self.analytical_tuner.get_analytical_gains()
            if self.use_analytical
            else None
        )

        # Pass results to trial tuner if we have them
        if self.results:
            self.trial_tuner.results = self.results

        self.trial_tuner.start(
            self.motor_interface,
            self.control_type,
            self.gravity_type,
            self.name,
            analytical_gains,
        )

    def _finalize_tuning(self):
        """Complete the tuning process and apply gains"""
        if self.results:
            # Select final gains (prefer trial if available, otherwise analytical)
            self.results.select_final_gains(use_trial=self.use_trial)

            # Print comparison
            self.results.print_comparison()

            # Apply final gains to motor
            self._apply_final_gains()

            # Stop motor
            self.motor_interface.set_control(ControlMode.BRAKE)

            self.state = TuningState.COMPLETE
        else:
            print(f"[{self.name}] Tuning failed - no results available")
            self.state = TuningState.FAILED

    def is_complete(self) -> bool:
        """Check if tuning is complete"""
        return self.state in (TuningState.COMPLETE, TuningState.FAILED)

    def get_final_gains(self) -> Optional[MotorGains]:
        """Get the final tuned gains after tuning is complete"""
        if self.state == TuningState.COMPLETE and self.results:
            return self.results.final_gains
        return None

    def _apply_final_gains(self):
        """Apply final tuned gains to the motor"""
        if self.results and self.results.final_gains:
            gains = self.results.final_gains
            success = self.motor_interface.apply_gains(
                kS=gains.kS,
                kV=gains.kV,
                kA=gains.kA,
                kP=gains.kP,
                kI=gains.kI,
                kD=gains.kD,
                kG=gains.kG,
            )

            if success:
                print(f"[{self.name}] Final gains applied to motor")
            else:
                print(f"[{self.name}] Warning: Could not apply final gains to motor")

    def get_results(self) -> Optional[MechanismTuningResults]:
        """Get the complete tuning results"""
        return self.results

    def export_gains_to_file(self, filename: str):
        """Export tuning results to a JSON file"""
        if self.results:
            import json

            with open(filename, "w") as f:
                json.dump(self.results.to_dict(), f, indent=2)
            print(f"[{self.name}] Gains exported to {filename}")

    def apply_gains(self, gains: MotorGains) -> bool:
        """Apply a set of gains to the motor"""
        return self.motor_interface.apply_gains(
            kS=gains.kS,
            kV=gains.kV,
            kA=gains.kA,
            kP=gains.kP,
            kI=gains.kI,
            kD=gains.kD,
            kG=gains.kG,
        )


# Convenience functions for common mechanisms


def tune_swerve_module(
    motor,
    name: str = "Swerve Module",
    position_getter: Optional[Callable[[], float]] = None,
) -> GenericMotorTuner:
    """
    Create a tuner for a swerve steering module.
    Call start_tuning() and periodic() on the returned tuner.

    Args:
        motor: TalonFX or SparkMax motor
        name: Name for the module
        position_getter: Optional custom position getter

    Returns:
        GenericMotorTuner configured for swerve module
    """
    return GenericMotorTuner(
        motor=motor,
        control_type=ControlType.POSITION,
        name=name,
        position_getter=position_getter,
    )


def tune_flywheel(motor, name: str = "Flywheel") -> GenericMotorTuner:
    """
    Create a tuner for a shooter flywheel.
    Call start_tuning() and periodic() on the returned tuner.

    Args:
        motor: TalonFX or SparkMax motor
        name: Name for the flywheel

    Returns:
        GenericMotorTuner configured for flywheel
    """
    return GenericMotorTuner(motor=motor, control_type=ControlType.VELOCITY, name=name)


def tune_hood(
    motor, name: str = "Hood", position_getter: Optional[Callable[[], float]] = None
) -> GenericMotorTuner:
    """
    Create a tuner for a shooter hood or similar mechanism.
    Call start_tuning(use_trial=False) and periodic() on the returned tuner for analytical tuning only.

    Args:
        motor: TalonFX or SparkMax motor
        name: Name for the hood
        position_getter: Optional custom position getter

    Returns:
        GenericMotorTuner configured for hood
    """
    return GenericMotorTuner(
        motor=motor,
        control_type=ControlType.POSITION,
        name=name,
        position_getter=position_getter,
    )


def tune_elevator(
    motor, name: str = "Elevator", position_getter: Optional[Callable[[], float]] = None
) -> GenericMotorTuner:
    """
    Create a tuner for an elevator with constant gravity compensation.
    Call start_tuning() and periodic() on the returned tuner.

    Args:
        motor: TalonFX or SparkMax motor
        name: Name for the elevator
        position_getter: Optional custom position getter (e.g., for height in meters)

    Returns:
        GenericMotorTuner configured for elevator
    """
    return GenericMotorTuner(
        motor=motor,
        control_type=ControlType.POSITION,
        gravity_type=GravityType.CONSTANT,
        name=name,
        position_getter=position_getter,
    )


def tune_arm(
    motor,
    name: str = "Arm Pivot",
    position_getter: Optional[Callable[[], float]] = None,
) -> GenericMotorTuner:
    """
    Create a tuner for an arm pivot with cosine gravity compensation.
    Call start_tuning() and periodic() on the returned tuner.

    Args:
        motor: TalonFX or SparkMax motor
        name: Name for the arm
        position_getter: Optional custom position getter (e.g., for angle in radians)

    Returns:
        GenericMotorTuner configured for arm
    """
    return GenericMotorTuner(
        motor=motor,
        control_type=ControlType.POSITION,
        gravity_type=GravityType.COSINE,
        name=name,
        position_getter=position_getter,
    )
