"""
Trial-and-error tuning component using empirical methods.
Incrementally tunes kS, kV, kP, and kD gains through observation.
"""
import math
from typing import List, Optional
import wpilib
from phoenix6 import controls
from phoenix6.configs import Slot0Configs
from wpilib import RobotBase, RuntimeType

from lemonlib.smart import SmartNT
from components.swerve_wheel import SwerveWheel
from components.tuning_data import ModuleTuningResults, TrialGains, AnalyticalGains


class TrialErrorTuner:
    """
    Component that performs trial-and-error (empirical) tuning.
    Incrementally adjusts gains based on system response.
    """
    
    def setup(self):
        self.nt = SmartNT("Trial Error Tuner")
        
        # Tuning increments
        self.kS_increment = 0.05  # Volts
        self.kV_increment = 0.01  # V/(rad/s)
        self.kP_increment = 0.5
        self.kD_increment = 0.01
        
        # State tracking
        self.is_running = False
        self.is_done = False
        self.current_module: Optional[SwerveWheel] = None
        self.results: Optional[ModuleTuningResults] = None
        
        # Current gain values being tested
        self.kS_trial = 0.0
        self.kV_trial = 0.0
        self.kP_trial = 0.0
        self.kD_trial = 0.0
        
        # Data collection
        self.position_samples: List[float] = []
        self.velocity_samples: List[float] = []
        self.time_samples: List[float] = []
        
        # Timing
        self.timer = wpilib.Timer()
        
        # Test state
        self.current_test = "idle"  # idle, kS, kV, kP, kD
        
        # kV tuning tracking
        self.velocity_setpoint = 2.0  # rad/s
        self.best_kV = 0.0
        self.best_kV_error = float('inf')
        
        # Config application tracking
        self.last_config_time = 0.0
        self.config_apply_delay = 0.1  # 100ms between updates
    
    """
    CONTROL METHODS
    """
    
    def start(self, module: SwerveWheel, initial_gains: Optional[AnalyticalGains] = None) -> None:
        """
        Start trial-and-error tuning for a module.
        
        Args:
            module: The SwerveWheel to tune
            initial_gains: Optional analytical gains to use as starting point
        """
        self.current_module = module
        self.is_running = True
        self.is_done = False
        
        module_id = module.direction_motor.device_id
        
        # Get or create results
        if self.results and self.results.module_id == module_id:
            # Continue with existing results
            pass
        else:
            self.results = ModuleTuningResults(module_id=module_id)
        
        # Initialize trial gains from analytical if provided
        if initial_gains:
            self.kS_trial = initial_gains.kS * 0.5  # Start conservative
            self.kV_trial = initial_gains.kV * 0.5
            self.kP_trial = initial_gains.kP * 0.3
            self.kD_trial = 0.0  # Always start kD from zero
        else:
            self.kS_trial = 0.0
            self.kV_trial = 0.0
            self.kP_trial = 0.0
            self.kD_trial = 0.0
        
        self.current_test = "kS"
        self.best_kV = 0.0
        self.best_kV_error = float('inf')
        self.timer.restart()
        self._reset_data()
        
        # Apply initial zero gains
        self._apply_motor_config(kS=0.0, kV=0.0, kP=0.0, kI=0.0, kD=0.0)
        
        print(f"Trial-and-error tuning started for module {module_id}")
    
    def stop(self) -> None:
        """Stop the current tuning process"""
        self.is_running = False
        if self.current_module:
            self.current_module.direction_motor.set_control(controls.StaticBrake())
    
    def is_complete(self) -> bool:
        """Check if tuning is complete"""
        return self.is_done
    
    def get_results(self) -> Optional[ModuleTuningResults]:
        """Get the tuning results"""
        return self.results
    
    def get_trial_gains(self) -> Optional[TrialGains]:
        """Get just the trial gains"""
        if self.results:
            return TrialGains(
                kS=self.results.trial_kS,
                kV=self.results.trial_kV,
                kP=self.results.trial_kP,
                kI=self.results.trial_kI,
                kD=self.results.trial_kD,
            )
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
        if self.current_module:
            position = self.current_module.getPosition()
            velocity = self.current_module.direction_motor.get_velocity().value
            
            self.position_samples.append(position.angle.radians())
            self.velocity_samples.append(velocity)
            self.time_samples.append(elapsed)
    
    def _apply_motor_config(self, **gains):
        """Apply motor configuration with rate limiting"""
        current_time = wpilib.Timer.getFPGATimestamp()
        
        if current_time - self.last_config_time < self.config_apply_delay:
            return
        
        slot_config = Slot0Configs()
        slot_config.k_s = gains.get('kS', 0.0)
        slot_config.k_v = gains.get('kV', 0.0)
        slot_config.k_p = gains.get('kP', 0.0)
        slot_config.k_i = gains.get('kI', 0.0)
        slot_config.k_d = gains.get('kD', 0.0)
        
        status = self.current_module.direction_motor.configurator.apply(slot_config, 0.050)
        
        if status.is_ok():
            self.last_config_time = current_time
            self.nt.put("Applied Config", True)
    
    def _calculate_velocity_error(self, target_velocity: float) -> float:
        """Calculate velocity tracking error"""
        if len(self.velocity_samples) > 10:
            avg_velocity = sum(abs(v) for v in self.velocity_samples[-10:]) / 10.0
            return abs(target_velocity - avg_velocity)
        return float('inf')
    
    def _detect_oscillation(self) -> bool:
        """Detect position oscillation"""
        if len(self.position_samples) < 30:
            return False
        
        recent_samples = self.position_samples[-30:]
        target = sum(recent_samples) / len(recent_samples)
        crossings = 0
        
        for i in range(1, len(recent_samples)):
            if (recent_samples[i-1] < target and recent_samples[i] > target) or \
               (recent_samples[i-1] > target and recent_samples[i] < target):
                crossings += 1
        
        return crossings >= 4
    
    def _detect_jitter(self) -> bool:
        """Detect velocity jitter"""
        if len(self.velocity_samples) < 30:
            return False
        
        recent_velocities = self.velocity_samples[-30:]
        mean_vel = sum(recent_velocities) / len(recent_velocities)
        variance = sum((v - mean_vel) ** 2 for v in recent_velocities) / len(recent_velocities)
        
        return variance > 0.5
    
    """
    TEST METHODS
    """
    
    def _tune_kS(self, elapsed: float):
        """Tune kS by increasing until movement detected"""
        self._collect_data(elapsed)
        
        # Apply kS voltage
        self.current_module.direction_motor.set_control(
            controls.VoltageOut(self.kS_trial)
        )
        
        self.nt.put("Trial kS", self.kS_trial)
        
        # Check for movement
        if len(self.position_samples) > 10:
            recent_movement = abs(
                self.position_samples[-1] - self.position_samples[-10]
            )
            
            if recent_movement > 0.01:  # Movement detected
                self.kS_trial = max(0.0, self.kS_trial - self.kS_increment)
                self.results.trial_kS = self.kS_trial
                self.nt.put(f"Module {self.results.module_id}/Trial kS", self.kS_trial)
                print(f"Module {self.results.module_id} - Trial kS: {self.kS_trial:.4f}")
                
                self.current_test = "kV"
                self._reset_data()
                self.best_kV = 0.0
                self.best_kV_error = float('inf')
                self._apply_motor_config(kS=self.kS_trial, kV=self.kV_trial, kP=0.0, kI=0.0, kD=0.0)
                return
        
        # Increment every 0.5 seconds
        if elapsed > 0.5 and int(elapsed / 0.5) != int((elapsed - 0.02) / 0.5):
            self.kS_trial += self.kS_increment
        
        # Timeout or simulation
        if elapsed > 15.0 or (RobotBase.getRuntimeType() == RuntimeType.kSimulation and elapsed > 2.0):
            if RobotBase.getRuntimeType() == RuntimeType.kSimulation:
                self.kS_trial = 0.5
            self.results.trial_kS = self.kS_trial
            print(f"Module {self.results.module_id} - Trial kS (timeout/sim): {self.kS_trial:.4f}")
            self.current_test = "kV"
            self._reset_data()
            self._apply_motor_config(kS=self.kS_trial, kV=self.kV_trial, kP=0.0, kI=0.0, kD=0.0)
    
    def _tune_kV(self, elapsed: float):
        """Tune kV by minimizing velocity tracking error"""
        self._collect_data(elapsed)
        
        # Use velocity control
        self.current_module.direction_motor.set_control(
            controls.VelocityVoltage(self.velocity_setpoint)
        )
        
        self.nt.put("Trial kV", self.kV_trial)
        
        # Check error after settling
        if elapsed > 1.0 and len(self.velocity_samples) > 20:
            error = self._calculate_velocity_error(self.velocity_setpoint)
            self.nt.put("kV Error", error)
            
            if error < self.best_kV_error:
                self.best_kV_error = error
                self.best_kV = self.kV_trial
            
            # If error increasing, we passed optimum
            if error > self.best_kV_error * 1.5 and self.kV_trial > self.best_kV + self.kV_increment:
                self.kV_trial = self.best_kV
                self.results.trial_kV = self.kV_trial
                self.nt.put(f"Module {self.results.module_id}/Trial kV", self.kV_trial)
                print(f"Module {self.results.module_id} - Trial kV: {self.kV_trial:.4f} (error: {self.best_kV_error:.4f})")
                
                self.current_test = "kP"
                self._reset_data()
                self._apply_motor_config(kS=self.kS_trial, kV=self.kV_trial, kP=self.kP_trial, kI=0.0, kD=0.0)
                return
        
        # Increment every 1.0 seconds
        if elapsed > 1.0 and int(elapsed) != int(elapsed - 0.02):
            self.kV_trial += self.kV_increment
            self._apply_motor_config(kS=self.kS_trial, kV=self.kV_trial, kP=0.0, kI=0.0, kD=0.0)
        
        # Timeout or simulation
        if elapsed > 20.0 or (RobotBase.getRuntimeType() == RuntimeType.kSimulation and elapsed > 2.0):
            if RobotBase.getRuntimeType() == RuntimeType.kSimulation:
                self.kV_trial = 0.12
            else:
                self.kV_trial = self.best_kV if self.best_kV > 0 else self.kV_trial
            self.results.trial_kV = self.kV_trial
            print(f"Module {self.results.module_id} - Trial kV (timeout/sim): {self.kV_trial:.4f}")
            self.current_test = "kP"
            self._reset_data()
            self._apply_motor_config(kS=self.kS_trial, kV=self.kV_trial, kP=self.kP_trial, kI=0.0, kD=0.0)
    
    def _tune_kP(self, elapsed: float):
        """Tune kP by increasing until oscillation"""
        self._collect_data(elapsed)
        
        # Use position control
        target = 0.0
        self.current_module.direction_motor.set_control(
            controls.PositionVoltage(target / math.tau)
        )
        
        self.nt.put("Trial kP", self.kP_trial)
        
        # Check for oscillation after settling
        if elapsed > 1.0 and len(self.position_samples) > 30:
            if self._detect_oscillation():
                self.kP_trial = max(0.0, self.kP_trial - self.kP_increment)
                self.results.trial_kP = self.kP_trial
                self.nt.put(f"Module {self.results.module_id}/Trial kP", self.kP_trial)
                print(f"Module {self.results.module_id} - Trial kP: {self.kP_trial:.4f}")
                
                self.current_test = "kD"
                self._reset_data()
                self._apply_motor_config(kS=self.kS_trial, kV=self.kV_trial, kP=self.kP_trial, kI=0.0, kD=self.kD_trial)
                return
        
        # Increment every 1.0 seconds
        if elapsed > 1.0 and int(elapsed) != int(elapsed - 0.02):
            self.kP_trial += self.kP_increment
            self._apply_motor_config(kS=self.kS_trial, kV=self.kV_trial, kP=self.kP_trial, kI=0.0, kD=0.0)
        
        # Timeout or simulation
        if elapsed > 20.0 or (RobotBase.getRuntimeType() == RuntimeType.kSimulation and elapsed > 2.0):
            if RobotBase.getRuntimeType() == RuntimeType.kSimulation:
                self.kP_trial = 10.0
            self.results.trial_kP = self.kP_trial
            print(f"Module {self.results.module_id} - Trial kP (timeout/sim): {self.kP_trial:.4f}")
            self.current_test = "kD"
            self._reset_data()
            self._apply_motor_config(kS=self.kS_trial, kV=self.kV_trial, kP=self.kP_trial, kI=0.0, kD=self.kD_trial)
    
    def _tune_kD(self, elapsed: float):
        """Tune kD by increasing until jitter"""
        self._collect_data(elapsed)
        
        # Use position control with full PD
        target = 0.0
        self.current_module.direction_motor.set_control(
            controls.PositionVoltage(target / math.tau)
        )
        
        self.nt.put("Trial kD", self.kD_trial)
        
        # Check for jitter after settling
        if elapsed > 1.0 and len(self.velocity_samples) > 30:
            if self._detect_jitter():
                self.kD_trial = max(0.0, self.kD_trial - self.kD_increment)
                self.results.trial_kD = self.kD_trial
                self.results.trial_kI = 0.0  # Always zero for position control
                self.nt.put(f"Module {self.results.module_id}/Trial kD", self.kD_trial)
                print(f"Module {self.results.module_id} - Trial kD: {self.kD_trial:.4f}")
                
                # Done!
                self.is_done = True
                self.is_running = False
                self.current_test = "idle"
                return
        
        # Increment every 1.0 seconds
        if elapsed > 1.0 and int(elapsed) != int(elapsed - 0.02):
            self.kD_trial += self.kD_increment
            self._apply_motor_config(kS=self.kS_trial, kV=self.kV_trial, kP=self.kP_trial, kI=0.0, kD=self.kD_trial)
        
        # Timeout or simulation
        if elapsed > 20.0 or (RobotBase.getRuntimeType() == RuntimeType.kSimulation and elapsed > 2.0):
            if RobotBase.getRuntimeType() == RuntimeType.kSimulation:
                self.kD_trial = 0.1
            self.results.trial_kD = self.kD_trial
            self.results.trial_kI = 0.0
            print(f"Module {self.results.module_id} - Trial kD (timeout/sim): {self.kD_trial:.4f}")
            self.is_done = True
            self.is_running = False
            self.current_test = "idle"
    
    """
    EXECUTE METHOD
    """
    
    def execute(self):
        """Called every robot loop - runs the current test"""
        if not self.is_running or self.current_module is None:
            return
        
        elapsed = self.timer.get()
        
        # Run the current test
        if self.current_test == "kS":
            self._tune_kS(elapsed)
        elif self.current_test == "kV":
            self._tune_kV(elapsed)
        elif self.current_test == "kP":
            self._tune_kP(elapsed)
        elif self.current_test == "kD":
            self._tune_kD(elapsed)
