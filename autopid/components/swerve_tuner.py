import math
from enum import Enum, auto
from typing import List, Dict, Optional
import wpilib
from phoenix6 import controls
from phoenix6.configs import Slot0Configs
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState
from components.swerve_wheel import SwerveWheel

from lemonlib.smart import SmartNT


class TuningState(Enum):
    """States for the auto-tuning state machine"""

    IDLE = 6
    FINDING_MIN_KP = 0
    FINDING_MAX_KP = 1
    OPTIMIZING_KP = 2
    ADDING_KD = 3
    VERIFICATION = 4
    COMPLETE = 5


class SwerveTuner:
    """
    Component for auto-tuning swerve module steering using automated trial-and-error.
    Mimics the manual tuning process but automated:
    1. Find minimum kP that responds
    2. Find maximum kP before oscillation
    3. Set kP to 80% of max
    4. Add kD to eliminate remaining oscillation
    """

    front_left: SwerveWheel

    def setup(self):
        self.nt = SmartNT("Swerve Tuner")
        self.state = TuningState.IDLE

        # Tuning parameters
        self.test_angles = [0, 22, 44, 66, 90]  # Degrees to test
        self.current_angle_index = 0

        # kP search parameters
        self.kP_min = 0.01
        self.kP_max = 200.0
        self.kP_step = 1.0
        self.current_kP = 0.01
        self.best_kP = 0.0
        self.oscillation_threshold = 0.05  # radians (~3 degrees)

        # kD search parameters
        self.kD_min = 0.0
        self.kD_max = 5.0
        self.kD_step = 0.1
        self.current_kD = 0.0
        self.best_kD = 0.0

        # Timing
        self.state_timer = wpilib.Timer()
        self.settle_time = 0.5  # Time to wait for settling
        self.test_duration = 1.5  # Time to observe behavior

        # Data collection
        self.position_samples: List[float] = []
        self.error_samples: List[float] = []
        self.time_samples: List[float] = []
        self.target_angle = 0.0

        # Results storage
        self.tuning_results: Dict[int, Dict] = {}

        # Current module being tuned
        self.front_left = None

        # Test phase tracking
        self.test_phase = 0  # Which angle we're testing

        self.target_state = SwerveModuleState(0.0, Rotation2d(0.0))

    def initialize_tuning(self):
        """
        Initialize the tuning process with a list of swerve modules.

        Args:
            modules: List of SwerveWheel components to tune
        """
        self.state = TuningState.FINDING_MIN_KP
        self.tuning_results.clear()

        self._start_state()

    def execute_tuning(self):
        """
        Main tuning execution - called periodically by the notifier.
        """
        if self.state == TuningState.IDLE or self.state == TuningState.COMPLETE:
            return

        if self.front_left is None:
            return

        # Execute current state
        if self.state == TuningState.FINDING_MIN_KP:
            self._execute_find_min_kp()
        elif self.state == TuningState.FINDING_MAX_KP:
            self._execute_find_max_kp()
        elif self.state == TuningState.OPTIMIZING_KP:
            self._execute_optimize_kp()
        elif self.state == TuningState.ADDING_KD:
            self._execute_add_kd()
        elif self.state == TuningState.VERIFICATION:
            self._execute_verification()

        # Always call module execute to process commands
        self.front_left.execute()

    def _start_state(self):
        """Initialize the current state"""
        self.state_timer.restart()
        self.position_samples.clear()
        self.error_samples.clear()
        self.time_samples.clear()
        self.current_angle_index = 0

        module_id = self.front_left.direction_motor.device_id
        self.nt.put(f"Current Module", module_id)
        self.nt.put(f"Current State", self.state.name)

        # Initialize tuning results for this module
        if module_id not in self.tuning_results:
            self.tuning_results[module_id] = {}

        print(f"Module {module_id}: Starting {self.state.name}")

    def _apply_pid_config(self, kP: float, kI: float, kD: float):
        """Apply PID configuration to the current module"""
        slot0 = Slot0Configs()
        slot0.k_p = kP
        slot0.k_i = kI
        slot0.k_d = kD

        # Apply the config to the motor
        self.front_left.direction_motor.configurator.apply(slot0)

        module_id = self.front_left.direction_motor.device_id
        self.nt.put(f"Module {module_id}/Current kP", kP)
        self.nt.put(f"Module {module_id}/Current kI", kI)
        self.nt.put(f"Module {module_id}/Current kD", kD)

        print(f"Applied: kP={kP:.1f}, kI={kI:.1f}, kD={kD:.2f}")

    def _command_angle(self, degrees: float):
        """Command the module to a specific angle using SwerveModuleState"""
        self.target_angle = math.radians(degrees)

        # Create a SwerveModuleState with the target angle and zero speed
        target_state = SwerveModuleState()
        target_state.angle = Rotation2d.fromDegrees(degrees)
        target_state.speed = 0.0  # Don't drive, just steer

        # Use the SwerveWheel's setDesiredState method
        self.target_state = target_state

    def _collect_data(self):
        """Collect position and error data"""
        elapsed = self.state_timer.get()

        # Get current position from the module
        position = self.front_left.getPosition()
        current_angle = position.angle.radians()

        # Calculate error
        error = self.target_angle - current_angle

        # Normalize error to [-pi, pi]
        while error > math.pi:
            error -= 2 * math.pi
        while error < -math.pi:
            error += 2 * math.pi

        self.position_samples.append(current_angle)
        self.error_samples.append(abs(error))
        self.time_samples.append(elapsed)

    def _is_oscillating(self) -> bool:
        """Detect if the system is oscillating"""
        if (
            len(self.error_samples) < 30
        ):  # Need at least 30 samples (0.6 seconds at 50Hz)
            return False

        # Look at recent error samples
        recent_errors = self.error_samples[-30:]

        # Simple method: check variance and mean
        mean_error = sum(recent_errors) / len(recent_errors)
        variance = sum((e - mean_error) ** 2 for e in recent_errors) / len(
            recent_errors
        )
        std_dev = math.sqrt(variance)

        # Oscillating if high variance and mean error is significant
        is_oscillating = (
            std_dev > 0.02 and mean_error > 0.02
        )  # ~1 degree variance and error

        if is_oscillating:
            print(
                f"  Oscillation detected! std_dev={math.degrees(std_dev):.2f}°, mean_error={math.degrees(mean_error):.2f}°"
            )

        return is_oscillating

    def _is_settled(self) -> bool:
        """Check if the system has settled to the target"""
        if len(self.error_samples) < 20:
            return False

        recent_errors = self.error_samples[-20:]
        max_recent_error = max(recent_errors)

        # Settled if error is consistently small
        return max_recent_error < 0.02  # ~1 degree

    def _is_responsive(self) -> bool:
        """Check if the system responded to the command"""
        if len(self.position_samples) < 20:
            return False

        # Check if position changed significantly
        position_change = abs(self.position_samples[-1] - self.position_samples[0])

        # Also check if it's moving toward target
        recent_errors = (
            self.error_samples[-10:]
            if len(self.error_samples) >= 10
            else self.error_samples
        )
        is_improving = len(recent_errors) > 1 and recent_errors[-1] < recent_errors[0]

        responsive = (
            position_change > 0.05 and is_improving
        )  # Moved more than ~3 degrees

        if not responsive:
            print(f"  Not responsive. Change: {math.degrees(position_change):.2f}°")

        return responsive

    def _execute_find_min_kp(self):
        """Find the minimum kP that makes the system responsive"""
        elapsed = self.state_timer.get()

        # First apply the current kP setting
        if elapsed < 0.1:
            self._apply_pid_config(self.current_kP, 0.0, 0.0)
            return

        # Command target angle
        if elapsed < 0.2:
            target_deg = self.test_angles[self.current_angle_index]
            print(f"Testing kP={self.current_kP:.1f}, commanding {target_deg}°")
            self._command_angle(target_deg)
            return

        # Collect data
        self._collect_data()

        # After test duration, evaluate
        if elapsed > self.test_duration:
            if self._is_responsive():
                # Found minimum kP!
                self.best_kP = self.current_kP
                module_id = self.front_left.direction_motor.device_id
                self.tuning_results[module_id]["min_kP"] = self.current_kP
                print(f"Found min kP = {self.current_kP:.1f}")

                # Move to next state
                self.current_kP = self.current_kP + self.kP_step
                self._transition_to_state(TuningState.FINDING_MAX_KP)
            else:
                # Not responsive yet, increase kP
                self.current_kP += self.kP_step

                if self.current_kP > self.kP_max:
                    # Failed to find responsive kP
                    print(f"ERROR: Could not find responsive kP up to {self.kP_max}")
                    self._transition_to_next_module()
                else:
                    # Try next angle
                    self.current_angle_index = (self.current_angle_index + 1) % len(
                        self.test_angles
                    )
                    self.state_timer.restart()
                    self.position_samples.clear()
                    self.error_samples.clear()
                    self.time_samples.clear()

    def _execute_find_max_kp(self):
        """Find the maximum kP before oscillation"""
        elapsed = self.state_timer.get()

        # Apply the current kP
        if elapsed < 0.1:
            self._apply_pid_config(self.current_kP, 0.0, 0.0)
            return

        # Command target angle
        if elapsed < 0.2:
            target_deg = self.test_angles[self.current_angle_index]
            print(f"Testing kP={self.current_kP:.1f}, commanding {target_deg}°")
            self._command_angle(target_deg)
            return

        # Collect data
        self._collect_data()

        # After test duration, evaluate
        if elapsed > self.test_duration:
            if self._is_oscillating():
                # Found oscillation! Max kP is previous value
                max_kP = self.current_kP - self.kP_step
                module_id = self.front_left.direction_motor.device_id
                self.tuning_results[module_id]["max_kP"] = max_kP
                print(
                    f"Found max kP = {max_kP:.1f} (oscillated at {self.current_kP:.1f})"
                )

                # Move to optimization
                self._transition_to_state(TuningState.OPTIMIZING_KP)
            else:
                # No oscillation yet, increase kP
                self.current_kP += self.kP_step

                if self.current_kP > self.kP_max:
                    # Hit limit without oscillation, use current value
                    module_id = self.front_left.direction_motor.device_id
                    self.tuning_results[module_id]["max_kP"] = self.current_kP
                    print(f"Reached kP limit {self.kP_max} without oscillation")
                    self._transition_to_state(TuningState.OPTIMIZING_KP)
                else:
                    # Try next angle
                    self.current_angle_index = (self.current_angle_index + 1) % len(
                        self.test_angles
                    )
                    self.state_timer.restart()
                    self.position_samples.clear()
                    self.error_samples.clear()
                    self.time_samples.clear()

    def _execute_optimize_kp(self):
        """Set kP to 80% of max for safety margin"""
        module_id = self.front_left.direction_motor.device_id
        results = self.tuning_results[module_id]

        # Use 80% of max kP
        optimal_kP = results["max_kP"] * 0.8
        self.best_kP = optimal_kP
        results["optimal_kP"] = optimal_kP

        print(f"Set optimal kP = {optimal_kP:.1f} (80% of max)")

        # Move to kD tuning
        self.current_kD = 0.0
        self._transition_to_state(TuningState.ADDING_KD)

    def _execute_add_kd(self):
        """Add kD to eliminate any remaining oscillation"""
        elapsed = self.state_timer.get()

        # Apply current PD values
        if elapsed < 0.1:
            self._apply_pid_config(self.best_kP, 0.0, self.current_kD)
            return

        # Command target angle
        if elapsed < 0.2:
            target_deg = self.test_angles[self.current_angle_index]
            print(f"Testing kD={self.current_kD:.2f}, commanding {target_deg}°")
            self._command_angle(target_deg)
            return

        # Collect data
        self._collect_data()

        # After test duration, evaluate
        if elapsed > self.test_duration:
            if self._is_oscillating():
                # Still oscillating, increase kD
                self.current_kD += self.kD_step

                if self.current_kD > self.kD_max:
                    # Hit kD limit
                    self.best_kD = self.kD_max
                    print(f"Reached kD limit {self.kD_max}, still some oscillation")
                    self._transition_to_state(TuningState.VERIFICATION)
                else:
                    # Try next angle with higher kD
                    self.current_angle_index = (self.current_angle_index + 1) % len(
                        self.test_angles
                    )
                    self.state_timer.restart()
                    self.position_samples.clear()
                    self.error_samples.clear()
                    self.time_samples.clear()
            else:
                # No oscillation! Found good kD
                self.best_kD = self.current_kD
                module_id = self.front_left.direction_motor.device_id
                self.tuning_results[module_id]["optimal_kD"] = self.current_kD
                print(f"Found optimal kD = {self.current_kD:.2f}")

                self._transition_to_state(TuningState.VERIFICATION)

    def _execute_verification(self):
        """Verify final tuning with multiple test moves"""
        elapsed = self.state_timer.get()

        # Apply final PD values
        if elapsed < 0.1:
            self._apply_pid_config(self.best_kP, 0.0, self.best_kD)
            return

        # Test all angles
        if self.current_angle_index < len(self.test_angles):
            # Command angle
            if elapsed < 0.3:
                target_deg = self.test_angles[self.current_angle_index]
                print(f"Verification: commanding {target_deg}°")
                self._command_angle(target_deg)
                return

            # Collect data
            self._collect_data()

            # Move to next angle after settle time
            if elapsed > 0.8:
                self.current_angle_index += 1
                self.state_timer.restart()
        else:
            # Verification complete
            module_id = self.front_left.direction_motor.device_id

            # Calculate average settling error
            avg_error = (
                sum(self.error_samples) / len(self.error_samples)
                if self.error_samples
                else 0
            )
            max_error = max(self.error_samples) if self.error_samples else 0

            self.tuning_results[module_id]["avg_error"] = avg_error
            self.tuning_results[module_id]["max_error"] = max_error

            # Store final values
            self.tuning_results[module_id]["final_kP"] = self.best_kP
            self.tuning_results[module_id]["final_kI"] = 0.0
            self.tuning_results[module_id]["final_kD"] = self.best_kD

            print(
                f"Verification complete - Avg error: {math.degrees(avg_error):.2f}°, Max error: {math.degrees(max_error):.2f}°"
            )

            # Move to next module
            self._transition_to_next_module()

    def _transition_to_state(self, new_state: TuningState):
        """Transition to a new state"""
        self.state = new_state
        self._start_state()

    def _transition_to_next_module(self):
        """Move to the next module or complete tuning"""
        # Stop current motor
        if self.front_left:
            self.front_left.stopped = True
            self.front_left.execute()

        # Move to next module
        self.front_left_index += 1

        if self.front_left_index < len(self.modules):
            # Tune next module
            self.front_left = self.modules[self.front_left_index]
            self.current_kP = self.kP_min
            self.current_kD = 0.0
            self.state = TuningState.FINDING_MIN_KP
            self._start_state()
        else:
            # All modules complete
            self.state = TuningState.COMPLETE

    def is_complete(self) -> bool:
        """Check if tuning is complete"""
        return self.state == TuningState.COMPLETE

    def finalize_tuning(self) -> Dict:
        """
        Finalize tuning and return results.

        Returns:
            Dictionary of tuning results for each module
        """
        # Stop all motors
        for module in self.modules:
            module.stopped = True
            module.execute()

        # Print summary
        print("\n===== Swerve Tuning Complete =====")
        for module_id, results in self.tuning_results.items():
            print(f"\nModule {module_id}:")
            print(f"  Final kP: {results.get('final_kP', 0):.2f}")
            print(f"  Final kI: {results.get('final_kI', 0):.2f}")
            print(f"  Final kD: {results.get('final_kD', 0):.2f}")
            print(f"  Min kP found: {results.get('min_kP', 0):.2f}")
            print(f"  Max kP found: {results.get('max_kP', 0):.2f}")
            print(f"  Avg error: {math.degrees(results.get('avg_error', 0)):.2f}°")
            print(f"  Max error: {math.degrees(results.get('max_error', 0)):.2f}°")
        print("\n==================================\n")

        self.state = TuningState.IDLE
        return self.tuning_results

    def execute(self):
        if self.state.value < 4:
            self.front_left.setDesiredState(self.target_state)
