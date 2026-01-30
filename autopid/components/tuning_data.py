"""
Shared data structures for swerve tuning components.
"""
from dataclasses import dataclass, field
from typing import Dict, Optional


@dataclass
class ModuleTuningResults:
    """Results for a single module's tuning process"""
    module_id: int
    
    # Static friction measurement
    static_friction_voltage: float = 0.0
    
    # Step response data
    time_constant_pos: Optional[float] = None
    time_constant_neg: Optional[float] = None
    max_velocity_pos: float = 0.0
    max_velocity_neg: float = 0.0
    
    # Oscillation test data
    oscillation_period: Optional[float] = None
    oscillation_amplitude: Optional[float] = None
    
    # Analytical gains
    analytical_kS: float = 0.0
    analytical_kV: float = 0.0
    analytical_kP: float = 0.0
    analytical_kI: float = 0.0
    analytical_kD: float = 0.0
    
    # Trial-and-error gains
    trial_kS: float = 0.0
    trial_kV: float = 0.0
    trial_kP: float = 0.0
    trial_kI: float = 0.0
    trial_kD: float = 0.0
    
    # Final selected gains
    kS: float = 0.0
    kV: float = 0.0
    kP: float = 0.0
    kI: float = 0.0
    kD: float = 0.0
    
    def select_final_gains(self, use_trial: bool = True):
        """
        Select final gains from either analytical or trial results.
        
        Args:
            use_trial: If True, use trial gains; otherwise use analytical
        """
        if use_trial:
            self.kS = self.trial_kS
            self.kV = self.trial_kV
            self.kP = self.trial_kP
            self.kI = self.trial_kI
            self.kD = self.trial_kD
        else:
            self.kS = self.analytical_kS
            self.kV = self.analytical_kV
            self.kP = self.analytical_kP
            self.kI = self.analytical_kI
            self.kD = self.analytical_kD
    
    def to_dict(self) -> Dict:
        """Convert to dictionary for NetworkTables or JSON export"""
        return {
            'module_id': self.module_id,
            'static_friction_voltage': self.static_friction_voltage,
            'time_constant_pos': self.time_constant_pos,
            'time_constant_neg': self.time_constant_neg,
            'max_velocity_pos': self.max_velocity_pos,
            'max_velocity_neg': self.max_velocity_neg,
            'oscillation_period': self.oscillation_period,
            'oscillation_amplitude': self.oscillation_amplitude,
            'analytical_kS': self.analytical_kS,
            'analytical_kV': self.analytical_kV,
            'analytical_kP': self.analytical_kP,
            'analytical_kI': self.analytical_kI,
            'analytical_kD': self.analytical_kD,
            'trial_kS': self.trial_kS,
            'trial_kV': self.trial_kV,
            'trial_kP': self.trial_kP,
            'trial_kI': self.trial_kI,
            'trial_kD': self.trial_kD,
            'kS': self.kS,
            'kV': self.kV,
            'kP': self.kP,
            'kI': self.kI,
            'kD': self.kD,
        }
    
    def print_comparison(self):
        """Print a comparison of analytical vs trial gains"""
        print(f"\n===== Module {self.module_id} Tuning Comparison =====")
        print(f"Analytical Method:")
        print(f"  kS: {self.analytical_kS:.4f}")
        print(f"  kV: {self.analytical_kV:.4f}")
        print(f"  kP: {self.analytical_kP:.4f}")
        print(f"  kI: {self.analytical_kI:.4f}")
        print(f"  kD: {self.analytical_kD:.4f}")
        print(f"Trial-and-Error Method:")
        print(f"  kS: {self.trial_kS:.4f}")
        print(f"  kV: {self.trial_kV:.4f}")
        print(f"  kP: {self.trial_kP:.4f}")
        print(f"  kI: {self.trial_kI:.4f}")
        print(f"  kD: {self.trial_kD:.4f}")
        print(f"Final Selected Gains:")
        print(f"  kS: {self.kS:.4f}")
        print(f"  kV: {self.kV:.4f}")
        print(f"  kP: {self.kP:.4f}")
        print(f"  kI: {self.kI:.4f}")
        print(f"  kD: {self.kD:.4f}")
        print(f"============================================\n")


@dataclass
class AnalyticalGains:
    """Analytical tuning gains calculated from system identification"""
    kS: float = 0.0
    kV: float = 0.0
    kP: float = 0.0
    kI: float = 0.0
    kD: float = 0.0


@dataclass
class TrialGains:
    """Trial-and-error tuning gains"""
    kS: float = 0.0
    kV: float = 0.0
    kP: float = 0.0
    kI: float = 0.0
    kD: float = 0.0
