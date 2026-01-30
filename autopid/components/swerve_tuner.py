"""
Main SwerveTuner state machine.
Orchestrates analytical and trial-and-error tuning components.
"""
from typing import List, Dict
from phoenix6 import controls
from phoenix6.configs import Slot0Configs
from magicbot import StateMachine, state, will_reset_to

from lemonlib.smart import SmartNT
from components.swerve_wheel import SwerveWheel
from components.analytical_tuner import AnalyticalTuner
from components.trial_error_tuner import TrialErrorTuner
from components.tuning_data import ModuleTuningResults


class SwerveTuner(StateMachine):
    """
    Main state machine for swerve module tuning.
    Coordinates analytical and trial-and-error tuning components.
    """
    
    # MagicBot will inject these components
    analytical_tuner: AnalyticalTuner
    trial_tuner: TrialErrorTuner
    
    # Swerve modules (injected by MagicBot)
    front_left: SwerveWheel
    # front_right: SwerveWheel
    # rear_left: SwerveWheel
    # rear_right: SwerveWheel
    
    # Command (will reset each loop)
    start_tuning_command = will_reset_to(False)
    
    def setup(self):
        # Always engage the state machine
        self.engage()
        
        self.nt = SmartNT("Swerve Tuner")
        
        # List of modules to tune
        self.modules: List[SwerveWheel] = [
            self.front_left,
            # self.front_right,
            # self.rear_left,
            # self.rear_right,
        ]
        
        # Current module being tuned
        self.current_module_index = 0
        self.current_module: SwerveWheel = None
        
        # Results storage
        self.tuning_results: Dict[int, ModuleTuningResults] = {}
        
        # Configuration application tracking
        self.last_config_time = 0.0
        self.config_apply_delay = 0.1
    
    """
    INFORMATIONAL METHODS
    """
    
    def is_tuning_active(self) -> bool:
        """Check if currently tuning (not in idle state)"""
        return self.current_state != "idle"
    
    def get_results(self) -> Dict[int, ModuleTuningResults]:
        """Get all tuning results"""
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
        """Prepare module for tuning"""
        module.stopped = True
        module.direction_motor.set_control(controls.StaticBrake())
        module.speed_motor.set_control(controls.StaticBrake())
        
        # Reset motor config to zero gains
        import wpilib
        current_time = wpilib.Timer.getFPGATimestamp()
        if current_time - self.last_config_time >= self.config_apply_delay:
            slot_config = Slot0Configs()
            slot_config.k_s = 0.0
            slot_config.k_v = 0.0
            slot_config.k_p = 0.0
            slot_config.k_i = 0.0
            slot_config.k_d = 0.0
            module.direction_motor.configurator.apply(slot_config, 0.050)
            self.last_config_time = current_time
    
    def _restore_module_after_tuning(self, module: SwerveWheel):
        """Restore module to normal operation with tuned gains"""
        module_id = module.direction_motor.device_id
        
        if module_id in self.tuning_results:
            results = self.tuning_results[module_id]
            
            # Apply final gains
            import wpilib
            current_time = wpilib.Timer.getFPGATimestamp()
            if current_time - self.last_config_time >= self.config_apply_delay:
                slot_config = Slot0Configs()
                slot_config.k_s = results.kS
                slot_config.k_v = results.kV
                slot_config.k_p = results.kP
                slot_config.k_i = results.kI
                slot_config.k_d = results.kD
                
                status = module.direction_motor.configurator.apply(slot_config, 0.050)
                if status.is_ok():
                    self.last_config_time = current_time
                    print(f"Applied final gains to module {module_id}")
        
        # Stop the module
        module.stopped = True
        module.direction_motor.set_control(controls.StaticBrake())
        module.speed_motor.set_control(controls.StaticBrake())
    
    """
    STATES
    """
    
    @state(first=True)
    def idle(self):
        """Idle state - waiting for tuning command"""
        if self.start_tuning_command:
            # Initialize tuning
            self.current_module_index = 0
            self.tuning_results.clear()
            
            if len(self.modules) > 0:
                self.current_module = self.modules[0]
                self._prepare_module_for_tuning(self.current_module)
                print(f"\n===== Starting Auto-Tune for {len(self.modules)} Modules =====\n")
                self.next_state('run_analytical_tuning')
    
    @state
    def run_analytical_tuning(self, initial_call):
        """Run analytical tuning component"""
        if initial_call:
            print(f"Starting analytical tuning for module {self.current_module.direction_motor.device_id}")
            self.analytical_tuner.start(self.current_module)
        
        # Check if analytical tuning is complete
        if self.analytical_tuner.is_complete():
            # Get results
            results = self.analytical_tuner.get_results()
            module_id = results.module_id
            
            # Store or update results
            if module_id not in self.tuning_results:
                self.tuning_results[module_id] = results
            else:
                # Merge analytical results into existing results
                self.tuning_results[module_id].static_friction_voltage = results.static_friction_voltage
                self.tuning_results[module_id].time_constant_pos = results.time_constant_pos
                self.tuning_results[module_id].time_constant_neg = results.time_constant_neg
                self.tuning_results[module_id].max_velocity_pos = results.max_velocity_pos
                self.tuning_results[module_id].max_velocity_neg = results.max_velocity_neg
                self.tuning_results[module_id].oscillation_period = results.oscillation_period
                self.tuning_results[module_id].oscillation_amplitude = results.oscillation_amplitude
                self.tuning_results[module_id].analytical_kS = results.analytical_kS
                self.tuning_results[module_id].analytical_kV = results.analytical_kV
                self.tuning_results[module_id].analytical_kP = results.analytical_kP
                self.tuning_results[module_id].analytical_kI = results.analytical_kI
                self.tuning_results[module_id].analytical_kD = results.analytical_kD
            
            print(f"Analytical tuning complete for module {module_id}")
            self.next_state('run_trial_tuning')
    
    @state
    def run_trial_tuning(self, initial_call):
        """Run trial-and-error tuning component"""
        if initial_call:
            print(f"Starting trial-and-error tuning for module {self.current_module.direction_motor.device_id}")
            
            # Get analytical gains to use as starting point
            analytical_gains = self.analytical_tuner.get_analytical_gains()
            
            # Pass results to trial tuner
            module_id = self.current_module.direction_motor.device_id
            self.trial_tuner.results = self.tuning_results[module_id]
            
            # Start trial tuning
            self.trial_tuner.start(self.current_module, analytical_gains)
        
        # Check if trial tuning is complete
        if self.trial_tuner.is_complete():
            # Results are already stored in self.tuning_results via shared reference
            results = self.tuning_results[self.current_module.direction_motor.device_id]
            
            print(f"Trial-and-error tuning complete for module {results.module_id}")
            self.next_state('finalize_module')
    
    @state
    def finalize_module(self, initial_call):
        """Finalize tuning for current module"""
        if not initial_call:
            return
        
        module_id = self.current_module.direction_motor.device_id
        results = self.tuning_results[module_id]
        
        # Select final gains (use trial by default)
        results.select_final_gains(use_trial=True)
        
        # Print comparison
        results.print_comparison()
        
        # Apply final gains and restore module
        self._restore_module_after_tuning(self.current_module)
        
        # Move to next module or finish
        self.next_state('advance_to_next_module')
    
    @state
    def advance_to_next_module(self, initial_call):
        """Advance to next module or complete tuning"""
        if not initial_call:
            return
        
        self.current_module_index += 1
        
        if self.current_module_index < len(self.modules):
            # Tune next module
            self.current_module = self.modules[self.current_module_index]
            self._prepare_module_for_tuning(self.current_module)
            self.next_state('run_analytical_tuning')
        else:
            # All modules complete
            self._finalize_all_tuning()
            self.next_state('idle')
    
    def _finalize_all_tuning(self):
        """Print final summary of all tuning results"""
        print("\n" + "="*60)
        print("SWERVE TUNING COMPLETE")
        print("="*60)
        
        for module_id, results in self.tuning_results.items():
            print(f"\nModule {module_id} - Final Gains:")
            print(f"  kS: {results.kS:.4f} V")
            print(f"  kV: {results.kV:.4f} V/(rad/s)")
            print(f"  kP: {results.kP:.4f}")
            print(f"  kI: {results.kI:.4f}")
            print(f"  kD: {results.kD:.4f}")
        
        print("\n" + "="*60 + "\n")
        
        # Output to NetworkTables
        for module_id, results in self.tuning_results.items():
            self.nt.put(f"Module {module_id}/Final kS", results.kS)
            self.nt.put(f"Module {module_id}/Final kV", results.kV)
            self.nt.put(f"Module {module_id}/Final kP", results.kP)
            self.nt.put(f"Module {module_id}/Final kI", results.kI)
            self.nt.put(f"Module {module_id}/Final kD", results.kD)
    
    def on_disable(self):
        """Handle robot disable"""
        if self.is_tuning_active():
            # Stop both tuners
            self.analytical_tuner.stop()
            self.trial_tuner.stop()
            
            # Stop all motors
            for module in self.modules:
                module.direction_motor.set_control(controls.StaticBrake())
                module.speed_motor.set_control(controls.StaticBrake())
                module.stopped = True
            
            # Return to idle
            self.next_state('idle')
        
        super().on_disable()
