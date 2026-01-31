"""
Shared data structures for generic motor tuning components.
"""

from dataclasses import dataclass, field
from typing import Dict, Optional
from enum import Enum


class ControlType(Enum):
    """Type of control for the mechanism"""

    POSITION = 0
    VELOCITY = 1


class GravityType(Enum):
    """Type of gravity compensation needed"""

    NONE = 0
    CONSTANT = 1
    COSINE = 2


class TuningProfile(Enum):
    """Ziegler-Nichols tuning profile variants"""

    PD = 0  # Pure PD: Fast response, good disturbance rejection, position only
    PI = 1  # Pure PI: Velocity control, integrator steady-state error elimination
    CLASSIC_PID = 2  # Classic PID: Balanced, ~25% overshoot, good disturbance rejection
    PESSEN_INTEGRAL = 3  # Pessen: Faster settling, ~15% overshoot
    NO_OVERSHOOT = 4  # No overshoot: Critical damping, slower settling
    SOME_OVERSHOOT = 5  # Some overshoot: Fast response, ~10% overshoot, balanced
    AUTO = 6  # Automatic selection based on application


@dataclass
class MotorGains:
    """Complete set of motor gains"""

    kS: float = 0.0  # Static friction feedforward (Volts)
    kV: float = 0.0  # Velocity feedforward (V/(unit/s))
    kA: float = 0.0  # Acceleration feedforward (V/(unit/s²))
    kP: float = 0.0  # Proportional gain
    kI: float = 0.0  # Integral gain
    kD: float = 0.0  # Derivative gain
    kG: float = 0.0  # Gravity feedforward (Volts)

    def to_dict(self) -> Dict:
        """Convert to dictionary for NetworkTables or JSON export"""
        return {
            "kS": self.kS,
            "kV": self.kV,
            "kA": self.kA,
            "kP": self.kP,
            "kI": self.kI,
            "kD": self.kD,
            "kG": self.kG,
        }

    def __str__(self) -> str:
        return (
            f"kS={self.kS:.4f}, kV={self.kV:.4f}, kA={self.kA:.4f}, "
            f"kP={self.kP:.4f}, kI={self.kI:.4f}, kD={self.kD:.4f}, kG={self.kG:.4f}"
        )


@dataclass
class MechanismTuningResults:
    """Results for a single mechanism's tuning process"""

    mechanism_id: int
    mechanism_name: str
    control_type: ControlType
    gravity_type: GravityType

    # Static friction measurement
    static_friction_voltage: float = 0.0

    # Step response data
    time_constant_pos: Optional[float] = None
    time_constant_neg: Optional[float] = None
    max_velocity_pos: float = 0.0
    max_velocity_neg: float = 0.0
    max_acceleration_pos: float = 0.0
    max_acceleration_neg: float = 0.0

    # Oscillation test data
    oscillation_period: Optional[float] = None
    oscillation_amplitude: Optional[float] = None

    # Gravity measurement (for CONSTANT and COSINE types)
    gravity_voltage: float = 0.0

    # Analytical gains
    analytical_gains: MotorGains = field(default_factory=MotorGains)

    # Trial-and-error gains
    trial_gains: MotorGains = field(default_factory=MotorGains)

    # Final selected gains
    final_gains: MotorGains = field(default_factory=MotorGains)

    def select_final_gains(self, use_trial: bool = True):
        """
        Select final gains from either analytical or trial results.

        Args:
            use_trial: If True, use trial gains; otherwise use analytical
        """
        if use_trial:
            self.final_gains = MotorGains(
                kS=self.trial_gains.kS,
                kV=self.trial_gains.kV,
                kA=self.trial_gains.kA,
                kP=self.trial_gains.kP,
                kI=self.trial_gains.kI,
                kD=self.trial_gains.kD,
                kG=self.trial_gains.kG,
            )
        else:
            self.final_gains = MotorGains(
                kS=self.analytical_gains.kS,
                kV=self.analytical_gains.kV,
                kA=self.analytical_gains.kA,
                kP=self.analytical_gains.kP,
                kI=self.analytical_gains.kI,
                kD=self.analytical_gains.kD,
                kG=self.analytical_gains.kG,
            )

    def to_dict(self) -> Dict:
        """Convert to dictionary for NetworkTables or JSON export"""
        return {
            "mechanism_id": self.mechanism_id,
            "mechanism_name": self.mechanism_name,
            "control_type": self.control_type.value,
            "gravity_type": self.gravity_type.value,
            "static_friction_voltage": self.static_friction_voltage,
            "time_constant_pos": self.time_constant_pos,
            "time_constant_neg": self.time_constant_neg,
            "max_velocity_pos": self.max_velocity_pos,
            "max_velocity_neg": self.max_velocity_neg,
            "max_acceleration_pos": self.max_acceleration_pos,
            "max_acceleration_neg": self.max_acceleration_neg,
            "oscillation_period": self.oscillation_period,
            "oscillation_amplitude": self.oscillation_amplitude,
            "gravity_voltage": self.gravity_voltage,
            "analytical_gains": self.analytical_gains.to_dict(),
            "trial_gains": self.trial_gains.to_dict(),
            "final_gains": self.final_gains.to_dict(),
        }

    def print_comparison(self):
        """Print a comparison of analytical vs trial gains"""
        print(
            f"\n===== {self.mechanism_name} (ID: {self.mechanism_id}) Tuning Comparison ====="
        )
        print(
            f"Control Type: {self.control_type.value}, Gravity Type: {self.gravity_type.value}"
        )
        print(f"\nAnalytical Method:")
        print(f"  {self.analytical_gains}")
        print(f"\nTrial-and-Error Method:")
        print(f"  {self.trial_gains}")
        print(f"\nFinal Selected Gains:")
        print(f"  {self.final_gains}")
        print(f"=" * 70 + "\n")
