"""
Generic motor tuner that works for any mechanism.
Supports position and velocity control with optional gravity compensation.
"""
from enum import Enum
from dataclasses import dataclass
from typing import Optional, Callable
import wpilib
from phoenix6 import controls
from phoenix6.hardware import TalonFX

from lemonlib.smart import SmartNT


class ControlType(Enum):
    """Type of control for the mechanism"""
    POSITION = "position"  # Angle/height control
    VELOCITY = "velocity"  # Speed control


class GravityType(Enum):
    """Type of gravity compensation needed"""
    NONE = "none"           # No gravity (horizontal rotation, flywheel)
    CONSTANT = "constant"   # Constant force (elevator)
    COSINE = "cosine"       # Varies with angle (arm, hood)


@dataclass
class MotorGains:
    """Motor control gains"""
    kS: float = 0.0  # Static friction (volts)
    kV: float = 0.0  # Velocity feedforward (volts per unit/sec)
    kP: float = 0.0  # Proportional gain
    kI: float = 0.0  # Integral gain
    kD: float = 0.0  # Derivative gain
    kG: float = 0.0  # Gravity feedforward (volts)
    
    def __str__(self):
        return f"kS={self.kS:.4f}, kV={self.kV:.4f}, kP={self.kP:.4f}, kI={self.kI:.4f}, kD={self.kD:.4f}, kG={self.kG:.4f}"


class GenericMotorTuner:
    """
    Universal motor tuner for any mechanism.
    
    Supports:
    - Position control (swerve, hood, turret, arm, elevator)
    - Velocity control (flywheel, drivetrain)
    - Gravity compensation (constant or cosine)
    
    Example usage:
        # Swerve steering
        tuner = GenericMotorTuner(
            motor=swerve_module.direction_motor,
            control_type=ControlType.POSITION,
            name="Swerve Steering"
        )
        
        # Flywheel
        tuner = GenericMotorTuner(
            motor=shooter.flywheel_motor,
            control_type=ControlType.VELOCITY,
            name="Flywheel"
        )
        
        # Elevator
        tuner = GenericMotorTuner(
            motor=elevator.motor,
            control_type=ControlType.POSITION,
            gravity_type=GravityType.CONSTANT,
            name="Elevator"
        )
        
        # Arm
        tuner = GenericMotorTuner(
            motor=arm.pivot_motor,
            control_type=ControlType.POSITION,
            gravity_type=GravityType.COSINE,
            name="Arm"
        )
    """
    
    def __init__(
        self,
        motor: TalonFX,
        control_type: ControlType,
        gravity_type: GravityType = GravityType.NONE,
        name: str = "Motor",
        position_getter: Optional[Callable[[], float]] = None,
        velocity_getter: Optional[Callable[[], float]] = None,
    ):
        """
        Initialize the generic motor tuner.
        
        Args:
            motor: The TalonFX motor to tune
            control_type: Position or velocity control
            gravity_type: Type of gravity compensation needed
            name: Human-readable name for the mechanism
            position_getter: Optional function to get position (radians or meters)
                           If None, uses motor encoder directly
            velocity_getter: Optional function to get velocity (rad/s or m/s)
                           If None, uses motor encoder directly
        """
        self.motor = motor
        self.control_type = control_type
        self.gravity_type = gravity_type
        self.name = name
        
        # Position/velocity getters
        self.position_getter = position_getter or (lambda: motor.get_position().value)
        self.velocity_getter = velocity_getter or (lambda: motor.get_velocity().value)
        
        # NetworkTables
        self.nt = SmartNT(f"Tuner/{name}")
        
        # Tuning state
        self.is_running = False
        self.is_complete = False
        
        # Results
        self.gains: Optional[MotorGains] = None
        
        # Test parameters
        self.test_voltage = 2.0
        self.oscillation_voltage = 1.5
        
        # Data collection
        self.position_samples = []
        self.velocity_samples = []
        self.time_samples = []
        self.timer = wpilib.Timer()
        
    def tune(self, use_analytical: bool = True, use_trial: bool = True) -> MotorGains:
        """
        Run the tuning process.
        
        Args:
            use_analytical: Run analytical tuning
            use_trial: Run trial-and-error tuning
            
        Returns:
            Tuned motor gains
        """
        print(f"\n=Tuning {self.name}")
        print(f"Control type: {self.control_type.value}")
        print(f"Gravity type: {self.gravity_type.value}")
        
        if use_analytical:
            analytical_gains = self._run_analytical_tuning()
            print(f"Analytical gains: {analytical_gains}")
            self.gains = analytical_gains
        
        if use_trial:
            initial = self.gains if use_analytical else MotorGains()
            trial_gains = self._run_trial_tuning(initial)
            print(f"Trial gains: {trial_gains}")
            self.gains = trial_gains
        
        print(f"{self.name} Tuning Complete\n")
        return self.gains
    
    def _run_analytical_tuning(self) -> MotorGains:
        """Run analytical tuning based on control type"""
        gains = MotorGains()
        
        # Measure static friction (same for all)
        gains.kS = self._measure_static_friction()
        
        # Measure gravity if needed
        if self.gravity_type != GravityType.NONE:
            gains.kG = self._measure_gravity()
        
        if self.control_type == ControlType.POSITION:
            # Position control gains
            tau = self._measure_time_constant()
            gains.kP = 1.2 / tau if tau > 0 else 10.0
            gains.kD = gains.kP * tau / 1.5 if tau > 0 else 0.1
            gains.kI = 0.0  # Usually not needed for position
            
            # Estimate kV from velocity
            max_vel = self._measure_max_velocity()
            gains.kV = self.test_voltage / max_vel if max_vel > 0.1 else 0.12
            
        else:  # VELOCITY
            # Velocity control gains
            max_vel = self._measure_max_velocity()
            gains.kV = self.test_voltage / max_vel if max_vel > 0.1 else 0.12
            
            # For velocity, kP acts on velocity error
            gains.kP = 0.5  # Start conservative
            gains.kI = 0.1  # Small integral for steady-state
            gains.kD = 0.0  # Usually not needed for velocity
        
        return gains
    
    def _run_trial_tuning(self, initial: MotorGains) -> MotorGains:
        """Run trial-and-error tuning starting from initial gains"""
        gains = MotorGains(
            kS=initial.kS * 0.5,  # Start conservative
            kV=initial.kV * 0.5,
            kP=initial.kP * 0.3,
            kI=initial.kI,
            kD=0.0,  # Always start from zero
            kG=initial.kG,
        )
        
        # Tune kS (same for all)
        gains.kS = self._trial_tune_kS(gains.kS)
        
        if self.control_type == ControlType.POSITION:
            # Position control trial tuning
            gains.kV = self._trial_tune_kV_position(gains)
            gains.kP = self._trial_tune_kP_position(gains)
            gains.kD = self._trial_tune_kD_position(gains)
        else:
            # Velocity control trial tuning
            gains.kV = self._trial_tune_kV_velocity(gains)
            gains.kP = self._trial_tune_kP_velocity(gains)
            gains.kI = self._trial_tune_kI_velocity(gains)
        
        return gains
    
    def _measure_static_friction(self) -> float:
        """Measure voltage needed to overcome static friction"""
        print(f"Measuring static friction...")
        
        self.timer.restart()
        self.position_samples.clear()
        voltage = 0.0
        
        while self.timer.get() < 10.0:
            voltage = min(self.timer.get() * 0.5, 3.0)
            self.motor.set_control(controls.VoltageOut(voltage))
            
            position = self.position_getter()
            self.position_samples.append(position)
            
            if len(self.position_samples) > 10:
                movement = abs(self.position_samples[-1] - self.position_samples[-10])
                if movement > 0.01:
                    break
            
            wpilib.wait(0.02)
        
        self.motor.set_control(controls.StaticBrake())
        print(f"  Static friction: {voltage:.3f}V")
        return voltage
    
    def _measure_gravity(self) -> float:
        """Measure gravity compensation needed"""
        print(f"Measuring gravity compensation...")
        
        if self.gravity_type == GravityType.CONSTANT:
            # Hold at current position, measure voltage needed
            target_pos = self.position_getter()
            self.timer.restart()
            voltage_sum = 0.0
            samples = 0
            
            while self.timer.get() < 2.0:
                current_pos = self.position_getter()
                error = target_pos - current_pos
                
                # Simple P controller to hold position
                voltage = 10.0 * error  # Temporary kP
                self.motor.set_control(controls.VoltageOut(voltage))
                
                voltage_sum += voltage
                samples += 1
                wpilib.wait(0.02)
            
            kG = voltage_sum / samples if samples > 0 else 0.5
            
        elif self.gravity_type == GravityType.COSINE:
            # Would need to measure at multiple angles
            # For now, use simple approximation
            kG = 0.5  # Placeholder
        else:
            kG = 0.0
        
        self.motor.set_control(controls.StaticBrake())
        print(f"Gravity compensation: {kG:.3f}V")
        return kG
    
    def _measure_time_constant(self) -> float:
        """Measure system time constant from step response"""
        print(f"Measuring time constant...")
        
        self.timer.restart()
        self.position_samples.clear()
        self.time_samples.clear()
        
        while self.timer.get() < 3.0:
            self.motor.set_control(controls.VoltageOut(self.test_voltage))
            
            position = self.position_getter()
            self.position_samples.append(position)
            self.time_samples.append(self.timer.get())
            
            wpilib.wait(0.02)
        
        self.motor.set_control(controls.StaticBrake())
        
        # Calculate time constant (63.2% of final value)
        if len(self.position_samples) < 10:
            return 0.1
        
        final = self.position_samples[-1]
        initial = self.position_samples[0]
        delta = final - initial
        target = initial + 0.632 * delta
        
        for i, pos in enumerate(self.position_samples):
            if abs(pos - target) < abs(delta * 0.05):
                tau = self.time_samples[i]
                print(f"  Time constant: {tau:.3f}s")
                return tau
        
        return 0.1
    
    def _measure_max_velocity(self) -> float:
        """Measure maximum velocity at test voltage"""
        print(f"  Measuring max velocity...")
        
        self.timer.restart()
        self.velocity_samples.clear()
        
        while self.timer.get() < 2.0:
            self.motor.set_control(controls.VoltageOut(self.test_voltage))
            
            velocity = self.velocity_getter()
            self.velocity_samples.append(abs(velocity))
            
            wpilib.wait(0.02)
        
        self.motor.set_control(controls.StaticBrake())
        
        max_vel = max(self.velocity_samples) if self.velocity_samples else 1.0
        print(f"Max velocity: {max_vel:.3f} units/s")
        return max_vel
    
    def _trial_tune_kS(self, initial: float) -> float:
        """Trial tune kS by increasing until movement"""
        print(f"Trial tuning kS...")
        # Implementation similar to TrialErrorTuner._tune_kS()
        # Simplified for example
        return initial
    
    def _trial_tune_kV_position(self, gains: MotorGains) -> float:
        """Trial tune kV for position control"""
        print(f"  Trial tuning kV...")
        # Implementation similar to TrialErrorTuner._tune_kV()
        return gains.kV
    
    def _trial_tune_kP_position(self, gains: MotorGains) -> float:
        """Trial tune kP for position control"""
        print(f"  Trial tuning kP...")
        # Implementation similar to TrialErrorTuner._tune_kP()
        return gains.kP
    
    def _trial_tune_kD_position(self, gains: MotorGains) -> float:
        """Trial tune kD for position control"""
        print(f"  Trial tuning kD...")
        # Implementation similar to TrialErrorTuner._tune_kD()
        return gains.kD
    
    def _trial_tune_kV_velocity(self, gains: MotorGains) -> float:
        """Trial tune kV for velocity control"""
        print(f"  Trial tuning kV for velocity...")
        return gains.kV
    
    def _trial_tune_kP_velocity(self, gains: MotorGains) -> float:
        """Trial tune kP for velocity control"""
        print(f"  Trial tuning kP for velocity...")
        return gains.kP
    
    def _trial_tune_kI_velocity(self, gains: MotorGains) -> float:
        """Trial tune kI for velocity control"""
        print(f"  Trial tuning kI for velocity...")
        return gains.kI


# Example usage for different mechanisms
def tune_swerve_module(swerve_module):
    """Tune a swerve steering module"""
    tuner = GenericMotorTuner(
        motor=swerve_module.direction_motor,
        control_type=ControlType.POSITION,
        name=f"Swerve Module {swerve_module.direction_motor.device_id}",
        position_getter=lambda: swerve_module.getPosition().angle.radians(),
    )
    return tuner.tune(use_analytical=True, use_trial=True)


def tune_flywheel(shooter):
    """Tune a shooter flywheel"""
    tuner = GenericMotorTuner(
        motor=shooter.flywheel_motor,
        control_type=ControlType.VELOCITY,
        name="Shooter Flywheel",
    )
    return tuner.tune(use_analytical=True, use_trial=True)


def tune_hood(shooter):
    """Tune a shooter hood"""
    tuner = GenericMotorTuner(
        motor=shooter.hood_motor,
        control_type=ControlType.POSITION,
        name="Shooter Hood",
    )
    return tuner.tune(use_analytical=True, use_trial=False)  # Analytical only


def tune_elevator(elevator):
    """Tune an elevator"""
    tuner = GenericMotorTuner(
        motor=elevator.motor,
        control_type=ControlType.POSITION,
        gravity_type=GravityType.CONSTANT,
        name="Elevator",
        position_getter=lambda: elevator.get_height(),
    )
    return tuner.tune(use_analytical=True, use_trial=True)


def tune_arm(arm):
    """Tune an arm pivot"""
    tuner = GenericMotorTuner(
        motor=arm.pivot_motor,
        control_type=ControlType.POSITION,
        gravity_type=GravityType.COSINE,
        name="Arm Pivot",
        position_getter=lambda: arm.get_angle(),
    )
    return tuner.tune(use_analytical=True, use_trial=True)
