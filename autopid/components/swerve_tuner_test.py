import math
from typing import List, Dict
import wpilib
from phoenix6 import controls
from magicbot import StateMachine, state, timed_state, will_reset_to

from lemonlib.smart import SmartNT
from components.swerve_wheel import SwerveWheel
from wpilib import RobotBase,RuntimeType


class SwerveTuner(StateMachine):
    """
    Component for auto-tuning swerve module steering PD values.
    Uses Ziegler-Nichols and step response methods.
    Note: Uses PD control (no integral) as it's more appropriate for position control.
    
    This component integrates with SwerveWheel components to tune their direction motors.
    During tuning, it takes direct control of the direction motors, bypassing normal
    SwerveWheel control logic.
    """
    front_left: SwerveWheel
    
    # Will reset to False each loop unless explicitly set
    start_tuning_command = will_reset_to(False)
    
    def setup(self):
        # Always engage the state machine (like ArmControl)
        self.engage()
        
        self.nt = SmartNT("Swerve Tuner")
        self.modules: List[SwerveWheel] = [
            self.front_left,
            # self.front_right,
            # self.rear_left,
            # self.rear_right,
        ]
        self.current_module_index = 0
        
        # Tuning parameters
        self.test_voltage = 2.0  # Volts for step response
        self.oscillation_voltage = 1.5  # Volts for oscillation test
        
        # Timing
        self.state_timer = wpilib.Timer()
        self.step_duration = 1.0  # seconds
        self.oscillation_duration = 3.0  # seconds
        
        # Data collection
        self.position_samples: List[float] = []
        self.time_samples: List[float] = []
        self.velocity_samples: List[float] = []
        
        # Results storage
        self.tuning_results: Dict[int, Dict] = {}
        
        # Current module being tuned
        self.current_module = None
        
        # Flag to track if we should process positive or negative step response
        self.processing_positive_step = True
        
    """
    INFORMATIONAL METHODS
    """
    
    def is_tuning_active(self) -> bool:
        """Check if currently tuning (not in idle state)"""
        return self.current_state != "idle"
    
    def get_results(self) -> Dict:
        """Get the tuning results"""
        return self.tuning_results
    
    """
    CONTROL METHODS
    """
        
    def start_tuning(self):
        """Command to start the tuning process"""
        self.start_tuning_command = True
    
    """
    HELPER METHODS
    """
    
    def _prepare_module_for_tuning(self, module: SwerveWheel):
        """
        Prepare a SwerveWheel module for tuning by stopping it and
        preventing normal execute() operation.
        """
        module.stopped = True
        module.direction_motor.set_control(controls.StaticBrake())
        module.speed_motor.set_control(controls.StaticBrake())
    
    def _restore_module_after_tuning(self, module: SwerveWheel):
        """
        Restore a SwerveWheel module to normal operation after tuning.
        """
        module_id = module.direction_motor.device_id
        
        if module_id in self.tuning_results:
            results = self.tuning_results[module_id]
            
            # Update the direction motor PID configuration
            if 'kP' in results:
                module.direction_motor_configs.slot0.k_p = results['kP']
                module.direction_motor_configs.slot0.k_i = results['kI']
                module.direction_motor_configs.slot0.k_d = results['kD']
                
                # Apply the configuration
                module.direction_motor.configurator.apply(
                    module.direction_motor_configs.slot0
                )
                
                print(f"Applied tuned gains to module {module_id}")
        
        # Stop the module (it will be controlled normally via setDesiredState)
        module.stopped = True
        module.direction_motor.set_control(controls.StaticBrake())
        module.speed_motor.set_control(controls.StaticBrake())
    
    def _reset_data_collection(self, initial_call):
        """Reset data collection arrays at the start of each state"""
        if initial_call:
            self.state_timer.restart()
            self.position_samples.clear()
            self.time_samples.clear()
            self.velocity_samples.clear()
            
            if self.current_module:
                module_id = self.current_module.direction_motor.device_id
                self.nt.put(f"Current Module", module_id)
    
    """
    STATES
    """
    
    @state(first=True)
    def idle(self):
        """
        Idle state - waiting for tuning command.
        Similar to ArmControl's homing state being first.
        """
        # Check if tuning was commanded
        if self.start_tuning_command:
            # Initialize tuning
            self.current_module_index = 0
            self.tuning_results.clear()
            self.processing_positive_step = True
            
            if len(self.modules) > 0:
                self.current_module = self.modules[0]
                self._prepare_module_for_tuning(self.current_module)
                print(f"Starting auto-tune for {len(self.modules)} swerve modules")
                self.next_state('measuring_static_friction')
    
    @state
    def measuring_static_friction(self, state_tm, initial_call):
        """
        Measure static friction by gradually increasing voltage until movement.
        """
        self._reset_data_collection(initial_call)
        
        if self.current_module is None:
            self.next_state('idle')
            return
        
        # Get current position from the SwerveWheel component
        position = self.current_module.getPosition()
        self.position_samples.append(position.angle.radians())
        self.time_samples.append(state_tm)
        
        # Gradually increase voltage
        voltage = min(state_tm * 0.5, 3.0)  # Ramp up to 3V over 6 seconds
        self.nt.put(f"Static Voltage", voltage)
        
        # Apply voltage directly to direction motor
        self.current_module.direction_motor.set_control(
            controls.VoltageOut(voltage)
        )
        
        module_id = self.current_module.direction_motor.device_id
        
        # Initialize results dict for this module if not exists
        if module_id not in self.tuning_results:
            self.tuning_results[module_id] = {}
        
        # Check if wheel started moving
        if len(self.position_samples) > 10:
            recent_movement = abs(
                self.position_samples[-1] - self.position_samples[-10]
            )
            if recent_movement > 0.01:  # ~0.57 degrees
                # Found static friction point
                static_friction_voltage = voltage
                
                self.tuning_results[module_id]['static_friction_voltage'] = static_friction_voltage
                self.nt.put(f"Module {module_id}/Static Friction V", static_friction_voltage)
                
                # Move to next state
                self.next_state('step_response')
                return
        
        # Timeout after 10 seconds
        if state_tm > 10.0:
            print(f"Warning: Static friction test timed out for module {module_id}")
            # Set a default value
            self.tuning_results[module_id]['static_friction_voltage'] = voltage
            self.next_state('step_response')

        if RobotBase.getRuntimeType() == RuntimeType.kSimulation:
            # In simulation, skip to next state after 2 seconds
            if state_tm > 2.0:
                static_friction_voltage = voltage
                self.tuning_results[module_id]['static_friction_voltage'] = static_friction_voltage
                self.nt.put(f"Module {module_id}/Static Friction V", static_friction_voltage)
                self.next_state('step_response')
    
    @timed_state(duration=1.0, next_state='analyze_step_response', must_finish=True)
    def step_response(self, state_tm, initial_call):
        """
        Execute step response test to measure system dynamics.
        Runs twice: once positive, once negative.
        """
        self._reset_data_collection(initial_call)
        
        if self.current_module is None:
            self.next_state('idle')
            return
        
        # Collect data
        position = self.current_module.getPosition()
        velocity = self.current_module.direction_motor.get_velocity().value
        
        self.position_samples.append(position.angle.radians())
        self.velocity_samples.append(velocity)
        self.time_samples.append(state_tm)
        
        # Apply step input based on which pass we're on
        voltage = self.test_voltage if self.processing_positive_step else -self.test_voltage
        self.current_module.direction_motor.set_control(
            controls.VoltageOut(voltage)
        )
    
    @state
    def analyze_step_response(self, initial_call):
        """
        Analyze collected step response data to extract system parameters.
        Determines whether to run another step response or move to oscillation test.
        """
        if not initial_call:
            return
            
        # Analyze data if we have enough samples
        if len(self.position_samples) >= 10:
            module_id = self.current_module.direction_motor.device_id
            
            # Ensure module entry exists
            if module_id not in self.tuning_results:
                self.tuning_results[module_id] = {}
            
            # Calculate velocity (derivative of position)
            max_velocity = max(abs(v) for v in self.velocity_samples)
            
            # Find time to reach 63.2% of final value (time constant)
            final_position = self.position_samples[-1]
            initial_position = self.position_samples[0]
            delta = final_position - initial_position
            target_63_2 = initial_position + 0.632 * delta
            
            time_constant = None
            for i, pos in enumerate(self.position_samples):
                if abs(pos - target_63_2) < abs(delta * 0.05):  # Within 5%
                    time_constant = self.time_samples[i]
                    break
            
            suffix = "pos" if self.processing_positive_step else "neg"
            
            if time_constant:
                self.tuning_results[module_id][f'time_constant_{suffix}'] = time_constant
                self.nt.put(f"Module {module_id}/Time Constant", time_constant)
            
            self.tuning_results[module_id][f'max_velocity_{suffix}'] = max_velocity
            self.nt.put(f"Module {module_id}/Max Velocity", max_velocity)
        
        # Determine next state based on which step we just completed
        if self.processing_positive_step:
            # Just finished positive step, now do negative
            self.processing_positive_step = False
            self.next_state('step_response')
        else:
            # Finished both steps, move to oscillation test
            self.processing_positive_step = True  # Reset for next module
            self.next_state('oscillation_test')
    
    @timed_state(duration=3.0, next_state='analyze_oscillations', must_finish=True)
    def oscillation_test(self, state_tm, initial_call):
        """
        Test for oscillations to find critical gain using relay feedback.
        """
        self._reset_data_collection(initial_call)
        
        if self.current_module is None:
            self.next_state('idle')
            return
        
        # Get current position
        position = self.current_module.getPosition()
        current_angle = position.angle.radians()
        
        self.position_samples.append(current_angle)
        self.time_samples.append(state_tm)
        
        # Simple relay: switch voltage based on error from setpoint
        target_angle = 0.0  # Try to hold at 0
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
    
    @state
    def analyze_oscillations(self, initial_call):
        """
        Analyze oscillation data to find ultimate gain and period.
        """
        # Only analyze data on initial call
        if initial_call and len(self.position_samples) >= 20:
            module_id = self.current_module.direction_motor.device_id
            
            # Ensure module entry exists
            if module_id not in self.tuning_results:
                self.tuning_results[module_id] = {}
            
            # Find peaks in position data
            peaks = []
            for i in range(1, len(self.position_samples) - 1):
                if (self.position_samples[i] > self.position_samples[i-1] and
                    self.position_samples[i] > self.position_samples[i+1]):
                    peaks.append(i)
            
            if len(peaks) >= 2:
                # Calculate average period between peaks
                periods = []
                for i in range(1, len(peaks)):
                    period = self.time_samples[peaks[i]] - self.time_samples[peaks[i-1]]
                    periods.append(period)
                
                avg_period = sum(periods) / len(periods)
                
                # Calculate amplitude
                amplitudes = [self.position_samples[p] for p in peaks]
                avg_amplitude = sum(amplitudes) / len(amplitudes)
                
                self.tuning_results[module_id]['oscillation_period'] = avg_period
                self.tuning_results[module_id]['oscillation_amplitude'] = avg_amplitude
                
                self.nt.put(f"Module {module_id}/Oscillation Period", avg_period)
                self.nt.put(f"Module {module_id}/Oscillation Amplitude", avg_amplitude)
        
        # Always transition to next state (even if analysis was skipped)
        if initial_call:
            self.next_state('fine_tuning')
    
    @timed_state(duration=0.5, next_state='advance_to_next_module', must_finish=True)
    def fine_tuning(self, initial_call):
        """
        Apply calculated PID values and verify performance.
        """
        if not initial_call:
            return
        
        if self.current_module is None:
            self.next_state('idle')
            return
        
        module_id = self.current_module.direction_motor.device_id
        
        # Ensure module entry exists
        if module_id not in self.tuning_results:
            self.tuning_results[module_id] = {}
        
        results = self.tuning_results[module_id]
        
        # Use oscillation data if available
        if 'oscillation_period' in results and 'oscillation_amplitude' in results:
            Ku = 4.0 * self.oscillation_voltage / (math.pi * results.get('oscillation_amplitude', 1.0))
            Tu = results['oscillation_period']
            
            # Modified Ziegler-Nichols for PD control (no integral for position control)
            kP = 0.45 * Ku  # Reduced from 0.6 for PD
            kI = 0.0  # No integral term for position control
            kD = 0.1 * Ku * Tu  # Slightly increased derivative
        else:
            # Fallback to step response
            tau = results.get('time_constant_pos', 0.1)
            kP = 1.2 / tau  # Slightly higher for PD
            kI = 0.0  # No integral term
            kD = kP * tau / 1.5  # Increased derivative contribution
        
        # Store calculated gains
        self.tuning_results[module_id]['kP'] = kP
        self.tuning_results[module_id]['kI'] = kI
        self.tuning_results[module_id]['kD'] = kD
        
        self.nt.put(f"Module {module_id}/Tuned kP", kP)
        self.nt.put(f"Module {module_id}/Tuned kI", kI)
        self.nt.put(f"Module {module_id}/Tuned kD", kD)
        
        print(f"Module {module_id} - Calculated PD: kP={kP:.4f}, kI={kI:.4f}, kD={kD:.4f}")
    
    @state
    def advance_to_next_module(self, initial_call):
        """
        Advance to the next module or complete the tuning process.
        """
        if not initial_call:
            return
        
        # Restore current module to normal operation
        if self.current_module:
            self._restore_module_after_tuning(self.current_module)
        
        # Move to next module
        self.current_module_index += 1
        
        if self.current_module_index < len(self.modules):
            # Tune next module - restart from the beginning
            self.current_module = self.modules[self.current_module_index]
            self._prepare_module_for_tuning(self.current_module)
            self.next_state('measuring_static_friction')
        else:
            # All modules complete
            self.finalize_tuning()
            # Return to idle state instead of calling done()
            self.next_state('idle')
    
    def on_disable(self):
        """
        Called when robot is disabled.
        Stop all tuning and restore modules.
        """
        if self.is_tuning_active():
            # Stop all motors
            for module in self.modules:
                module.direction_motor.set_control(controls.StaticBrake())
                module.speed_motor.set_control(controls.StaticBrake())
                module.stopped = True
            
            # Return to idle state
            self.next_state('idle')
        
        super().on_disable()
    
    def finalize_tuning(self) -> Dict:
        """
        Finalize tuning and return results.
        
        Returns:
            Dictionary of tuning results for each module
        """
        # Print summary
        print("\n===== Swerve Tuning Complete =====")
        for module_id, results in self.tuning_results.items():
            print(f"\nModule {module_id}:")
            for key, value in results.items():
                print(f"  {key}: {value:.4f}")
        print("\n==================================\n")
        
        return self.tuning_results
    