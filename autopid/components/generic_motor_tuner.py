"""
Generic motor tuner that works for any mechanism.
Supports position and velocity control with optional gravity compensation.
"""
from enum import Enum
from dataclasses import dataclass
from typing import Optional, Callable, List, Any
import wpilib
from phoenix6 import controls
from phoenix6.configs import Slot0Configs
from phoenix6.hardware import TalonFX

from lemonlib.smart import SmartNT
from components.swerve_wheel import SwerveWheel


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
    kA: float = 0.0  # Acceleration feedforward (volts per unit/sec^2)
    kP: float = 0.0  # Proportional gain
    kI: float = 0.0  # Integral gain
    kD: float = 0.0  # Derivative gain
    kG: float = 0.0  # Gravity feedforward (volts)
    
    def __str__(self) -> str:
        return f"kS={self.kS:.4f}, kV={self.kV:.4f}, kA={self.kA:.4f}, kP={self.kP:.4f}, kI={self.kI:.4f}, kD={self.kD:.4f}, kG={self.kG:.4f}"


class GenericMotorTuner:
    """
    Universal motor tuner for any mechanism.
    
    Supports:
    - Position control (swerve, hood, turret, arm, elevator)
    - Velocity control (flywheel, drivetrain)
    - Gravity compensation (constant or cosine)
    
    Example usage:
    ```
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
        ```
    """
    
    def __init__(
        self,
        motor: TalonFX,
        control_type: ControlType,
        gravity_type: GravityType = GravityType.NONE,
        name: str = "Motor",
        position_getter: Optional[Callable[[], float]] = None,
        velocity_getter: Optional[Callable[[], float]] = None,
    ) -> None:
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
        self.position_samples: List[float] = []
        self.velocity_samples: List[float] = []
        self.time_samples: List[float] = []
        self.timer = wpilib.Timer()
        
        # Config application tracking
        self.last_config_time = 0.0
        self.config_apply_delay = 0.1  # 100ms between updates
        
    def tune(self, use_analytical: bool = True, use_trial: bool = True) -> MotorGains:
        """
        Run the tuning process.
        
        Args:
            use_analytical: Run analytical tuning
            use_trial: Run trial-and-error tuning
            
        Returns:
            Tuned motor gains
        """
        print(f"\nTuning {self.name}")
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
            
            # Measure kA from acceleration
            gains.kA = self._measure_acceleration_gain()
            
        else:  # VELOCITY
            # Velocity control gains
            max_vel = self._measure_max_velocity()
            gains.kV = self.test_voltage / max_vel if max_vel > 0.1 else 0.12
            
            # Measure kA from acceleration
            gains.kA = self._measure_acceleration_gain()
            
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
            kA=initial.kA,
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
            gains.kA = self._trial_tune_kA_position(gains)
            gains.kP = self._trial_tune_kP_position(gains)
            gains.kD = self._trial_tune_kD_position(gains)
        else:
            # Velocity control trial tuning
            gains.kV = self._trial_tune_kV_velocity(gains)
            gains.kA = self._trial_tune_kA_velocity(gains)
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
    
    def _measure_acceleration_gain(self) -> float:
        """Measure acceleration feedforward gain"""
        print(f"  Measuring acceleration gain...")
        
        self.timer.restart()
        self.velocity_samples.clear()
        self.time_samples.clear()
        
        # Apply voltage and measure acceleration
        while self.timer.get() < 1.5:
            self.motor.set_control(controls.VoltageOut(self.test_voltage))
            
            velocity = self.velocity_getter()
            self.velocity_samples.append(abs(velocity))
            self.time_samples.append(self.timer.get())
            
            wpilib.wait(0.02)
        
        self.motor.set_control(controls.StaticBrake())
        
        # Calculate average acceleration
        if len(self.velocity_samples) > 20:
            # Use linear regression for acceleration
            n = len(self.velocity_samples)
            sum_t = sum(self.time_samples)
            sum_v = sum(self.velocity_samples)
            sum_tv = sum(t * v for t, v in zip(self.time_samples, self.velocity_samples))
            sum_t2 = sum(t * t for t in self.time_samples)
            
            # Slope = acceleration
            acceleration = (n * sum_tv - sum_t * sum_v) / (n * sum_t2 - sum_t * sum_t)
            kA = self.test_voltage / acceleration if abs(acceleration) > 0.1 else 0.01
            print(f"  Acceleration: {acceleration:.3f} units/s^2, kA: {kA:.4f}")
            return max(0.0, min(kA, 0.1))  # Clamp between 0 and 0.1
        
        return 0.01
    
    def _apply_motor_config(self, **gains) -> None:
        """Apply motor configuration with rate limiting"""
        current_time = wpilib.Timer.getFPGATimestamp()
        
        if current_time - self.last_config_time < self.config_apply_delay:
            return
        
        slot_config = Slot0Configs()
        slot_config.k_s = gains.get('kS', 0.0)
        slot_config.k_v = gains.get('kV', 0.0)
        slot_config.k_a = gains.get('kA', 0.0)
        slot_config.k_p = gains.get('kP', 0.0)
        slot_config.k_i = gains.get('kI', 0.0)
        slot_config.k_d = gains.get('kD', 0.0)
        
        status = self.motor.configurator.apply(slot_config, 0.050)
        
        if status.is_ok():
            self.last_config_time = current_time
            self.nt.put("Applied Config", True)
    
    def _detect_oscillation(self, target: float) -> bool:
        """Detect position oscillation around target"""
        if len(self.position_samples) < 30:
            return False
        
        recent_samples = self.position_samples[-30:]
        crossings = 0
        
        for i in range(1, len(recent_samples)):
            if (recent_samples[i-1] < target and recent_samples[i] > target) or \
               (recent_samples[i-1] > target and recent_samples[i] < target):
                crossings += 1
        
        return crossings >= 4
    
    def _detect_velocity_oscillation(self) -> bool:
        """Detect velocity oscillation"""
        if len(self.velocity_samples) < 30:
            return False
        
        recent_samples = self.velocity_samples[-30:]
        mean_vel = sum(recent_samples) / len(recent_samples)
        crossings = 0
        
        for i in range(1, len(recent_samples)):
            if (recent_samples[i-1] < mean_vel and recent_samples[i] > mean_vel) or \
               (recent_samples[i-1] > mean_vel and recent_samples[i] < mean_vel):
                crossings += 1
        
        return crossings >= 4
    
    def _detect_jitter(self) -> bool:
        """Detect velocity jitter (high variance)"""
        if len(self.velocity_samples) < 30:
            return False
        
        recent_velocities = self.velocity_samples[-30:]
        mean_vel = sum(recent_velocities) / len(recent_velocities)
        variance = sum((v - mean_vel) ** 2 for v in recent_velocities) / len(recent_velocities)
        
        return variance > 0.5
    
    def _trial_tune_kS(self, initial: float) -> float:
        """Trial tune kS by increasing until movement"""
        print(f"Trial tuning kS...")
        
        kS = initial
        self.timer.restart()
        self.position_samples.clear()
        
        while self.timer.get() < 15.0:
            elapsed = self.timer.get()
            
            # Apply voltage
            self.motor.set_control(controls.VoltageOut(kS))
            
            # Collect position data
            position = self.position_getter()
            self.position_samples.append(position)
            
            # Check for movement
            if len(self.position_samples) > 10:
                recent_movement = abs(
                    self.position_samples[-1] - self.position_samples[-10]
                )
                
                if recent_movement > 0.01:  # Movement detected
                    kS = max(0.0, kS - 0.05)  # Back off slightly
                    break
            
            # Increment every 0.5 seconds
            if elapsed > 0.5 and int(elapsed / 0.5) != int((elapsed - 0.02) / 0.5):
                kS += 0.05
            
            wpilib.wait(0.02)
        
        self.motor.set_control(controls.StaticBrake())
        print(f"  Final kS: {kS:.4f}V")
        return kS
    
    def _trial_tune_kV_position(self, gains: MotorGains) -> float:
        """Trial tune kV for position control by testing velocity tracking"""
        print(f"  Trial tuning kV for position...")
        
        kV = gains.kV
        best_kV = kV
        best_error = float('inf')
        velocity_setpoint = 2.0  # rad/s or m/s
        
        # Apply gains to motor
        config = controls.MotionMagicVelocityVoltage(velocity_setpoint)
        config.slot = 0
        
        self.timer.restart()
        
        while self.timer.get() < 20.0:
            elapsed = self.timer.get()
            
            # Apply velocity control
            self.motor.set_control(controls.VelocityVoltage(velocity_setpoint))
            
            # Collect velocity data
            velocity = self.velocity_getter()
            self.velocity_samples.append(abs(velocity))
            
            # Calculate error after settling
            if elapsed > 1.0 and len(self.velocity_samples) > 20:
                avg_velocity = sum(self.velocity_samples[-20:]) / 20.0
                error = abs(velocity_setpoint - avg_velocity)
                
                if error < best_error:
                    best_error = error
                    best_kV = kV
                
                # If error increasing significantly, we passed optimum
                if error > best_error * 1.5 and kV > best_kV + 0.01:
                    kV = best_kV
                    break
            
            # Increment every 1.0 seconds
            if elapsed > 1.0 and int(elapsed) != int(elapsed - 0.02):
                kV += 0.01
                self._apply_motor_config(kS=gains.kS, kV=kV, kA=gains.kA, kP=0.0, kI=0.0, kD=0.0)
                self.velocity_samples.clear()
            
            wpilib.wait(0.02)
        
        self.motor.set_control(controls.StaticBrake())
        kV = best_kV if best_kV > 0 else kV
        print(f"  Final kV: {kV:.4f} (error: {best_error:.4f})")
        return kV
    
    def _trial_tune_kA_position(self, gains: MotorGains) -> float:
        """Trial tune kA for position control by testing acceleration tracking"""
        print(f"  Trial tuning kA for position...")
        
        kA = gains.kA
        best_kA = kA
        best_error = float('inf')
        
        self.timer.restart()
        
        for test_iteration in range(10):
            self.velocity_samples.clear()
            self.timer.restart()
            
            # Apply motion profile with acceleration
            while self.timer.get() < 1.0:
                # Quick acceleration test
                self.motor.set_control(controls.VoltageOut(self.test_voltage))
                
                velocity = self.velocity_getter()
                self.velocity_samples.append(abs(velocity))
                wpilib.wait(0.02)
            
            # Measure tracking error during acceleration
            if len(self.velocity_samples) > 10:
                # Calculate acceleration achieved
                v_initial = sum(self.velocity_samples[:5]) / 5
                v_final = sum(self.velocity_samples[-5:]) / 5
                accel = (v_final - v_initial) / 1.0
                
                # Expected vs actual (simplified)
                expected_accel = self.test_voltage / (gains.kV + kA) if gains.kV > 0 else 1.0
                error = abs(expected_accel - accel) / max(expected_accel, 0.1)
                
                if error < best_error:
                    best_error = error
                    best_kA = kA
            
            # Increment kA
            kA += 0.005
            self._apply_motor_config(kS=gains.kS, kV=gains.kV, kA=kA, kP=0.0, kI=0.0, kD=0.0)
            
            if kA > 0.1:  # Max limit
                break
        
        self.motor.set_control(controls.StaticBrake())
        kA = best_kA if best_kA > 0 else gains.kA
        print(f"  Final kA: {kA:.4f}")
        return kA
    
    def _trial_tune_kP_position(self, gains: MotorGains) -> float:
        """Trial tune kP for position control by increasing until oscillation"""
        print(f"  Trial tuning kP for position...")
        
        kP = gains.kP
        target_position = self.position_getter()
        
        self.timer.restart()
        self.position_samples.clear()
        
        while self.timer.get() < 20.0:
            elapsed = self.timer.get()
            
            # Use position control
            self.motor.set_control(controls.PositionVoltage(target_position))
            
            # Collect position data
            position = self.position_getter()
            self.position_samples.append(position)
            
            # Check for oscillation after settling
            if elapsed > 1.0 and len(self.position_samples) > 30:
                if self._detect_oscillation(target_position):
                    kP = max(0.0, kP - 0.5)  # Back off
                    break
            
            # Increment every 1.0 seconds
            if elapsed > 1.0 and int(elapsed) != int(elapsed - 0.02):
                kP += 0.5
                self._apply_motor_config(kS=gains.kS, kV=gains.kV, kA=gains.kA, kP=kP, kI=0.0, kD=0.0)
                self.position_samples.clear()
            
            wpilib.wait(0.02)
        
        self.motor.set_control(controls.StaticBrake())
        print(f"  Final kP: {kP:.4f}")
        return kP
    
    def _trial_tune_kD_position(self, gains: MotorGains) -> float:
        """Trial tune kD for position control by increasing until jitter"""
        print(f"  Trial tuning kD for position...")
        
        kD = 0.0
        target_position = self.position_getter()
        
        self.timer.restart()
        self.velocity_samples.clear()
        
        while self.timer.get() < 20.0:
            elapsed = self.timer.get()
            
            # Use position control with full PD
            self.motor.set_control(controls.PositionVoltage(target_position))
            
            # Collect velocity data
            velocity = self.velocity_getter()
            self.velocity_samples.append(velocity)
            
            # Check for jitter after settling
            if elapsed > 1.0 and len(self.velocity_samples) > 30:
                if self._detect_jitter():
                    kD = max(0.0, kD - 0.01)  # Back off
                    break
            
            # Increment every 1.0 seconds
            if elapsed > 1.0 and int(elapsed) != int(elapsed - 0.02):
                kD += 0.01
                self._apply_motor_config(kS=gains.kS, kV=gains.kV, kA=gains.kA, kP=gains.kP, kI=0.0, kD=kD)
                self.velocity_samples.clear()
            
            wpilib.wait(0.02)
        
        self.motor.set_control(controls.StaticBrake())
        print(f"  Final kD: {kD:.4f}")
        return kD
    
    def _trial_tune_kV_velocity(self, gains: MotorGains) -> float:
        """Trial tune kV for velocity control by minimizing steady-state error"""
        print(f"  Trial tuning kV for velocity...")
        
        kV = gains.kV
        best_kV = kV
        best_error = float('inf')
        velocity_setpoint = 5.0  # rad/s or m/s
        
        self.timer.restart()
        
        while self.timer.get() < 20.0:
            elapsed = self.timer.get()
            
            # Apply velocity control
            self.motor.set_control(controls.VelocityVoltage(velocity_setpoint))
            
            # Collect velocity data
            velocity = self.velocity_getter()
            self.velocity_samples.append(abs(velocity))
            
            # Calculate error after settling
            if elapsed > 1.5 and len(self.velocity_samples) > 20:
                avg_velocity = sum(self.velocity_samples[-20:]) / 20.0
                error = abs(velocity_setpoint - avg_velocity)
                
                if error < best_error:
                    best_error = error
                    best_kV = kV
                
                # If error increasing significantly, we passed optimum
                if error > best_error * 1.5 and kV > best_kV + 0.01:
                    kV = best_kV
                    break
            
            # Increment every 1.0 seconds
            if elapsed > 1.5 and int(elapsed) != int(elapsed - 0.02):
                kV += 0.01
                self._apply_motor_config(kS=gains.kS, kV=kV, kA=gains.kA, kP=0.0, kI=0.0, kD=0.0)
                self.velocity_samples.clear()
            
            wpilib.wait(0.02)
        
        self.motor.set_control(controls.StaticBrake())
        kV = best_kV if best_kV > 0 else kV
        print(f"  Final kV: {kV:.4f} (error: {best_error:.4f})")
        return kV
    
    def _trial_tune_kA_velocity(self, gains: MotorGains) -> float:
        """Trial tune kA for velocity control by testing acceleration response"""
        print(f"  Trial tuning kA for velocity...")
        
        kA = gains.kA
        best_kA = kA
        best_error = float('inf')
        
        self.timer.restart()
        
        for test_iteration in range(10):
            self.velocity_samples.clear()
            self.timer.restart()
            
            # Test acceleration response
            while self.timer.get() < 1.0:
                # Ramp velocity command
                target_vel = self.timer.get() * 5.0  # Accelerate to 5 units/s
                self.motor.set_control(controls.VelocityVoltage(target_vel))
                
                velocity = self.velocity_getter()
                self.velocity_samples.append(abs(velocity))
                wpilib.wait(0.02)
            
            # Measure tracking during acceleration
            if len(self.velocity_samples) > 10:
                # Average error during ramp
                error = 0
                for i, v in enumerate(self.velocity_samples):
                    expected = (i * 0.02) * 5.0  # Expected velocity at this time
                    error += abs(expected - v)
                error /= len(self.velocity_samples)
                
                if error < best_error:
                    best_error = error
                    best_kA = kA
            
            # Increment kA
            kA += 0.005
            self._apply_motor_config(kS=gains.kS, kV=gains.kV, kA=kA, kP=0.0, kI=0.0, kD=0.0)
            
            if kA > 0.1:  # Max limit
                break
        
        self.motor.set_control(controls.StaticBrake())
        kA = best_kA if best_kA > 0 else gains.kA
        print(f"  Final kA: {kA:.4f} (error: {best_error:.4f})")
        return kA
    
    def _trial_tune_kP_velocity(self, gains: MotorGains) -> float:
        """Trial tune kP for velocity control by increasing until oscillation"""
        print(f"  Trial tuning kP for velocity...")
        
        kP = gains.kP
        velocity_setpoint = 5.0  # rad/s or m/s
        
        self.timer.restart()
        self.velocity_samples.clear()
        
        while self.timer.get() < 20.0:
            elapsed = self.timer.get()
            
            # Apply velocity control
            self.motor.set_control(controls.VelocityVoltage(velocity_setpoint))
            
            # Collect velocity data
            velocity = self.velocity_getter()
            self.velocity_samples.append(velocity)
            
            # Check for oscillation after settling
            if elapsed > 1.0 and len(self.velocity_samples) > 30:
                if self._detect_velocity_oscillation():
                    kP = max(0.0, kP - 0.1)  # Back off
                    break
            
            # Increment every 1.0 seconds
            if elapsed > 1.0 and int(elapsed) != int(elapsed - 0.02):
                kP += 0.1
                self._apply_motor_config(kS=gains.kS, kV=gains.kV, kA=gains.kA, kP=kP, kI=0.0, kD=0.0)
                self.velocity_samples.clear()
            
            wpilib.wait(0.02)
        
        self.motor.set_control(controls.StaticBrake())
        print(f"  Final kP: {kP:.4f}")
        return kP
    
    def _trial_tune_kI_velocity(self, gains: MotorGains) -> float:
        """Trial tune kI for velocity control by improving steady-state error"""
        print(f"  Trial tuning kI for velocity...")
        
        kI = 0.0
        best_kI = 0.0
        best_error = float('inf')
        velocity_setpoint = 5.0  # rad/s or m/s
        
        self.timer.restart()
        
        while self.timer.get() < 15.0:
            elapsed = self.timer.get()
            
            # Apply velocity control
            self.motor.set_control(controls.VelocityVoltage(velocity_setpoint))
            
            # Collect velocity data
            velocity = self.velocity_getter()
            self.velocity_samples.append(abs(velocity))
            
            # Calculate steady-state error after settling
            if elapsed > 2.0 and len(self.velocity_samples) > 30:
                avg_velocity = sum(self.velocity_samples[-30:]) / 30.0
                error = abs(velocity_setpoint - avg_velocity)
                
                if error < best_error:
                    best_error = error
                    best_kI = kI
                
                # If error increasing or system unstable, stop
                if error > best_error * 1.5 or error > 0.5:
                    kI = best_kI
                    break
            
            # Increment every 1.0 seconds
            if elapsed > 2.0 and int(elapsed) != int(elapsed - 0.02):
                kI += 0.05
                self._apply_motor_config(kS=gains.kS, kV=gains.kV, kA=gains.kA, kP=gains.kP, kI=kI, kD=0.0)
                self.velocity_samples.clear()
            
            wpilib.wait(0.02)
        
        self.motor.set_control(controls.StaticBrake())
        kI = best_kI if best_kI > 0 else 0.0
        print(f"  Final kI: {kI:.4f} (error: {best_error:.4f})")
        return kI


# Example usage for different mechanisms
def tune_swerve_module(swerve_module: SwerveWheel) -> MotorGains:
    """Tune a swerve steering module"""
    tuner = GenericMotorTuner(
        motor=swerve_module.direction_motor,
        control_type=ControlType.POSITION,
        name=f"Swerve Module {swerve_module.direction_motor.device_id}",
        position_getter=lambda: swerve_module.getPosition().angle.radians(),
    )
    return tuner.tune(use_analytical=True, use_trial=True)


def tune_flywheel(shooter: Any) -> MotorGains:
    """Tune a shooter flywheel"""
    tuner = GenericMotorTuner(
        motor=shooter.flywheel_motor,
        control_type=ControlType.VELOCITY,
        name="Shooter Flywheel",
    )
    return tuner.tune(use_analytical=True, use_trial=True)


def tune_hood(shooter: Any) -> MotorGains:
    """Tune a shooter hood"""
    tuner = GenericMotorTuner(
        motor=shooter.hood_motor,
        control_type=ControlType.POSITION,
        name="Shooter Hood",
    )
    return tuner.tune(use_analytical=True, use_trial=False)  # Analytical only


def tune_elevator(elevator: Any) -> MotorGains:
    """Tune an elevator"""
    tuner = GenericMotorTuner(
        motor=elevator.motor,
        control_type=ControlType.POSITION,
        gravity_type=GravityType.CONSTANT,
        name="Elevator",
        position_getter=lambda: elevator.get_height(),
    )
    return tuner.tune(use_analytical=True, use_trial=True)


def tune_arm(arm: Any) -> MotorGains:
    """Tune an arm pivot"""
    tuner = GenericMotorTuner(
        motor=arm.pivot_motor,
        control_type=ControlType.POSITION,
        gravity_type=GravityType.COSINE,
        name="Arm Pivot",
        position_getter=lambda: arm.get_angle(),
    )
    return tuner.tune(use_analytical=True, use_trial=True)
