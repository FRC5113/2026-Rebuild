"""
Shared data structures for generic motor tuning components.
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, Optional


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

    kP: float = 0.0  # Proportional gain
    kI: float = 0.0  # Integral gain
    kD: float = 0.0  # Derivative gain

    def to_dict(self) -> Dict:
        """Convert to dictionary for NetworkTables or JSON export"""
        return {
            "kP": self.kP,
            "kI": self.kI,
            "kD": self.kD,
        }

    def __str__(self) -> str:
        return f"kP={self.kP:.4f}, kI={self.kI:.4f}, kD={self.kD:.4f}"


@dataclass
class FeedforwardGains:
    """Feedforward gains (optional)"""

    kS: float = 0.0  # Static friction feedforward
    kV: float = 0.0  # Velocity feedforward
    kA: float = 0.0  # Acceleration feedforward
    kG: float = 0.0  # Gravity feedforward

    def to_dict(self) -> Dict:
        return {
            "kS": self.kS,
            "kV": self.kV,
            "kA": self.kA,
            "kG": self.kG,
        }


@dataclass
class MechanismTuningResults:
    """Results for a single mechanism's tuning process"""

    mechanism_id: int
    mechanism_name: str
    control_type: ControlType
    gravity_type: GravityType

    # Step response data
    time_constant_pos: Optional[float] = None
    time_constant_neg: Optional[float] = None

    # Oscillation test data
    oscillation_period: Optional[float] = None
    oscillation_amplitude: Optional[float] = None

    # Analytical gains
    analytical_gains: MotorGains = field(default_factory=MotorGains)

    # Trial-and-error gains
    trial_gains: MotorGains = field(default_factory=MotorGains)

    # Final selected gains
    final_gains: MotorGains = field(default_factory=MotorGains)

    # Feedforward gains used during tuning (optional)
    feedforward_gains: FeedforwardGains = field(default_factory=FeedforwardGains)

    def select_final_gains(self, use_trial: bool = True):
        """
        Select final gains from either analytical or trial results.

        Args:
            use_trial: If True, use trial gains; otherwise use analytical
        """
        if use_trial:
            self.final_gains = MotorGains(
                kP=self.trial_gains.kP,
                kI=self.trial_gains.kI,
                kD=self.trial_gains.kD,
            )
        else:
            self.final_gains = MotorGains(
                kP=self.analytical_gains.kP,
                kI=self.analytical_gains.kI,
                kD=self.analytical_gains.kD,
            )

    def to_dict(self) -> Dict:
        """Convert to dictionary for NetworkTables or JSON export"""
        return {
            "mechanism_id": self.mechanism_id,
            "mechanism_name": self.mechanism_name,
            "control_type": self.control_type.value,
            "gravity_type": self.gravity_type.value,
            "time_constant_pos": self.time_constant_pos,
            "time_constant_neg": self.time_constant_neg,
            "oscillation_period": self.oscillation_period,
            "oscillation_amplitude": self.oscillation_amplitude,
            "analytical_gains": self.analytical_gains.to_dict(),
            "trial_gains": self.trial_gains.to_dict(),
            "final_gains": self.final_gains.to_dict(),
            "feedforward_gains": self.feedforward_gains.to_dict(),
        }

    def print_comparison(self):
        """Print a comparison of analytical vs trial gains"""
        print(
            f"\n===== {self.mechanism_name} (ID: {self.mechanism_id}) Tuning Comparison ====="
        )
        print(
            f"Control Type: {self.control_type.value}, Gravity Type: {self.gravity_type.value}"
        )
        print("\nAnalytical Method:")
        print(f"  {self.analytical_gains}")
        print("\nTrial-and-Error Method:")
        print(f"  {self.trial_gains}")
        print("\nFinal Selected Gains:")
        print(f"  {self.final_gains}")
        print("=" * 70 + "\n")
