"""
Analytical tuning component using physics-based methods.
Performs static friction measurement, step response tests, and Ziegler-Nichols tuning.
"""
import math
from typing import List, Optional
import wpilib
from phoenix6 import controls
from wpilib import RobotBase, RuntimeType

from lemonlib.smart import SmartNT
from components.swerve_wheel import SwerveWheel
from components.tuning_data import ModuleTuningResults, AnalyticalGains


class AnalyticalTuner:
    """
    Component that performs analytical (physics-based) tuning.
    Uses step response and oscillation tests to calculate optimal PID gains.
    """
    
    def setup(self):
        self.nt = SmartNT("Analytical Tuner")
        
        # Tuning parameters
        self.test_voltage = 2.0  # Volts for step response
        self.oscillation_voltage = 1.5  # Volts for oscillation test
        
        # State tracking
        self.is_running = False
        self.is_done = False
        self.current_module: Optional[SwerveWheel] = None
        self.results: Optional[ModuleTuningResults] = None
        
        # Data collection
        self.position_samples: List[float] = []
        self.time_samples: List[float] = []
        self.velocity_samples: List[float] = []
        
        # Timing
        self.timer = wpilib.Timer()
        
        # Test state
        self.current_test = "idle"  # idle, static_friction, step_pos, step_neg, oscillation
        self.processing_positive_step = True
        
    """
    CONTROL METHODS
    """
    
    def start(self, module: SwerveWheel) -> None:
        """
        Start analytical tuning for a module.
        
        Args:
            module: The SwerveWheel to tune
        """
        self.current_module = module
        self.is_running = True
        self.is_done = False
        
        module_id = module.direction_motor.device_id
        self.results = ModuleTuningResults(module_id=module_id)
        
        self.current_test = "static_friction"
        self.processing_positive_step = True
        self.timer.restart()
        self._reset_data()
        
        print(f"Analytical tuning started for module {module_id}")
    
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
    
    def get_analytical_gains(self) -> Optional[AnalyticalGains]:
        """Get just the analytical gains"""
        if self.results:
            return AnalyticalGains(
                kS=self.results.analytical_kS,
                kV=self.results.analytical_kV,
                kP=self.results.analytical_kP,
                kI=self.results.analytical_kI,
                kD=self.results.analytical_kD,
            )
        return None
    
    """
    HELPER METHODS
    """
    
    def _reset_data(self):
        """Clear data collection arrays"""
        self.position_samples.clear()
        self.time_samples.clear()
        self.velocity_samples.clear()
        self.timer.restart()
    
    def _collect_data(self, elapsed: float):
        """Collect position and velocity data"""
        if self.current_module:
            position = self.current_module.getPosition()
            velocity = self.current_module.direction_motor.get_velocity().value
            
            self.position_samples.append(position.angle.radians())
            self.velocity_samples.append(velocity)
            self.time_samples.append(elapsed)
    
    """
    TEST METHODS
    """
    
    def _test_static_friction(self, elapsed: float):
        """Measure static friction by gradually increasing voltage"""
        self._collect_data(elapsed)
        
        # Gradually increase voltage
        voltage = min(elapsed * 0.5, 3.0)  # Ramp up to 3V over 6 seconds
        self.nt.put("Static Voltage", voltage)
        
        self.current_module.direction_motor.set_control(
            controls.VoltageOut(voltage)
        )
        
        # Check if wheel started moving
        if len(self.position_samples) > 10:
            recent_movement = abs(
                self.position_samples[-1] - self.position_samples[-10]
            )
            if recent_movement > 0.01:  # ~0.57 degrees
                # Found static friction point
                self.results.static_friction_voltage = voltage
                self.nt.put(f"Module {self.results.module_id}/Static Friction V", voltage)
                print(f"Module {self.results.module_id} - Static friction: {voltage:.3f}V")
                
                # Move to step response
                self.current_test = "step_pos"
                self.processing_positive_step = True
                self._reset_data()
                return
        
        # Timeout or simulation mode
        if elapsed > 10.0 or (RobotBase.getRuntimeType() == RuntimeType.kSimulation and elapsed > 2.0):
            self.results.static_friction_voltage = voltage
            self.nt.put(f"Module {self.results.module_id}/Static Friction V", voltage)
            print(f"Module {self.results.module_id} - Static friction (timeout/sim): {voltage:.3f}V")
            self.current_test = "step_pos"
            self.processing_positive_step = True
            self._reset_data()
    
    def _test_step_response(self, elapsed: float, positive: bool):
        """Execute step response test"""
        self._collect_data(elapsed)
        
        # Apply step input
        voltage = self.test_voltage if positive else -self.test_voltage
        self.current_module.direction_motor.set_control(
            controls.VoltageOut(voltage)
        )
        
        self.nt.put("Step Voltage", voltage)
        
        # Duration: 3 seconds
        if elapsed > 3.0:
            # Analyze the step response
            self._analyze_step_response(positive)
            
            # Move to next test
            if positive:
                self.current_test = "step_neg"
                self.processing_positive_step = False
            else:
                self.current_test = "oscillation"
                self.processing_positive_step = True
            
            self._reset_data()
    
    def _analyze_step_response(self, positive: bool):
        """Analyze collected step response data"""
        if len(self.position_samples) < 10 or len(self.velocity_samples) < 10:
            return
        
        # Calculate max velocity
        max_velocity = max(abs(v) for v in self.velocity_samples)
        
        # Find time constant (63.2% of final value)
        final_position = self.position_samples[-1]
        initial_position = self.position_samples[0]
        delta = final_position - initial_position
        
        time_constant = None
        if abs(delta) > 0.001:
            target_63_2 = initial_position + 0.632 * delta
            
            for i, pos in enumerate(self.position_samples):
                if abs(pos - target_63_2) < abs(delta * 0.05):  # Within 5%
                    time_constant = self.time_samples[i]
                    break
        
        # Store results
        suffix = "pos" if positive else "neg"
        if positive:
            self.results.time_constant_pos = time_constant
            self.results.max_velocity_pos = max_velocity
        else:
            self.results.time_constant_neg = time_constant
            self.results.max_velocity_neg = max_velocity
        
        self.nt.put(f"Module {self.results.module_id}/Time Constant {suffix}", time_constant or 0.0)
        self.nt.put(f"Module {self.results.module_id}/Max Velocity {suffix}", max_velocity)
        
        print(f"Module {self.results.module_id} - Step response ({suffix}): "
              f"tau={time_constant:.3f}s, max_vel={max_velocity:.3f}" if time_constant else 
              f"max_vel={max_velocity:.3f}")
    
    def _test_oscillation(self, elapsed: float):
        """Test for oscillations using relay feedback"""
        self._collect_data(elapsed)
        
        # Simple relay: switch voltage based on error from setpoint
        current_angle = self.position_samples[-1] if self.position_samples else 0.0
        target_angle = 0.0
        error = target_angle - current_angle
        
        # Relay with hysteresis
        if error > 0.05:  # ~3 degrees
            voltage = self.oscillation_voltage
        elif error < -0.05:
            voltage = -self.oscillation_voltage
        else:
            voltage = 0.0
        
        self.current_module.direction_motor.set_control(
            controls.VoltageOut(voltage)
        )
        
        self.nt.put("Oscillation Voltage", voltage)
        
        # Duration: 3 seconds
        if elapsed > 3.0:
            self._analyze_oscillations()
            self._calculate_analytical_gains()
            self.is_done = True
            self.is_running = False
            self.current_test = "idle"
    
    def _analyze_oscillations(self):
        """Analyze oscillation data to find period and amplitude"""
        if len(self.position_samples) < 20:
            return
        
        # Find peaks in position data
        peaks = []
        for i in range(1, len(self.position_samples) - 1):
            if (self.position_samples[i] > self.position_samples[i-1] and
                self.position_samples[i] > self.position_samples[i+1]):
                peaks.append(i)
        
        if len(peaks) >= 2:
            # Calculate average period
            periods = []
            for i in range(1, len(peaks)):
                period = self.time_samples[peaks[i]] - self.time_samples[peaks[i-1]]
                periods.append(period)
            
            avg_period = sum(periods) / len(periods)
            
            # Calculate amplitude
            amplitudes = [self.position_samples[p] for p in peaks]
            avg_amplitude = sum(amplitudes) / len(amplitudes)
            
            self.results.oscillation_period = avg_period
            self.results.oscillation_amplitude = avg_amplitude
            
            self.nt.put(f"Module {self.results.module_id}/Oscillation Period", avg_period)
            self.nt.put(f"Module {self.results.module_id}/Oscillation Amplitude", avg_amplitude)
            
            print(f"Module {self.results.module_id} - Oscillations: "
                  f"period={avg_period:.3f}s, amplitude={avg_amplitude:.3f}rad")
    
    def _calculate_analytical_gains(self):
        """Calculate PID gains using Ziegler-Nichols method"""
        # Use oscillation data if available
        if self.results.oscillation_period and self.results.oscillation_amplitude:
            amplitude = self.results.oscillation_amplitude
            if amplitude > 0.001:
                Ku = 4.0 * self.oscillation_voltage / (math.pi * amplitude)
                Tu = self.results.oscillation_period
                
                # Modified Ziegler-Nichols for PD control
                kP = 0.45 * Ku
                kI = 0.0
                kD = 0.1 * Ku * Tu
            else:
                kP = 10.0
                kI = 0.0
                kD = 0.1
        else:
            # Fallback to step response
            tau = self.results.time_constant_pos or 0.1
            if tau > 0:
                kP = 1.2 / tau
                kD = kP * tau / 1.5
            else:
                kP = 10.0
                kD = 0.1
            kI = 0.0
        
        # Estimate kS from static friction
        kS = self.results.static_friction_voltage
        
        # Estimate kV from max velocity
        max_vel = max(
            self.results.max_velocity_pos,
            self.results.max_velocity_neg
        )
        kV = self.test_voltage / max_vel if max_vel > 0.1 else 0.12
        
        # Store analytical gains
        self.results.analytical_kS = kS
        self.results.analytical_kV = kV
        self.results.analytical_kP = kP
        self.results.analytical_kI = kI
        self.results.analytical_kD = kD
        
        self.nt.put(f"Module {self.results.module_id}/Analytical kS", kS)
        self.nt.put(f"Module {self.results.module_id}/Analytical kV", kV)
        self.nt.put(f"Module {self.results.module_id}/Analytical kP", kP)
        self.nt.put(f"Module {self.results.module_id}/Analytical kI", kI)
        self.nt.put(f"Module {self.results.module_id}/Analytical kD", kD)
        
        print(f"Module {self.results.module_id} - Analytical gains: "
              f"kS={kS:.4f}, kV={kV:.4f}, kP={kP:.4f}, kI={kI:.4f}, kD={kD:.4f}")
    
    """
    EXECUTE METHOD
    """
    
    def execute(self):
        """Called every robot loop - runs the current test"""
        if not self.is_running or self.current_module is None:
            return
        
        elapsed = self.timer.get()
        
        # Run the current test
        if self.current_test == "static_friction":
            self._test_static_friction(elapsed)
        elif self.current_test == "step_pos":
            self._test_step_response(elapsed, positive=True)
        elif self.current_test == "step_neg":
            self._test_step_response(elapsed, positive=False)
        elif self.current_test == "oscillation":
            self._test_oscillation(elapsed)
