"""
Comprehensive examples for using the Generic Motor Tuner.

This module demonstrates how to tune various FRC mechanisms including:
- Swerve modules
- Flywheels
- Hoods
- Elevators
- Arms
- And more!

All examples use the new non-blocking state machine API that integrates with RobotPy's periodic loop.
"""

from .generic_motor_tuner import (
    GenericMotorTuner,
    tune_arm,
    tune_elevator,
    tune_flywheel,
    tune_hood,
    tune_swerve_module,
)
from .tuning_data import ControlType, FeedforwardGains, GravityType, MotorGains


def example_tune_swerve_module_simple(swerve_module):
    """
    Simple example: Create a swerve module tuner using the convenience function.
    Call start_tuning() once, then call periodic() in your robot loop.
    """
    # Assuming swerve_module has a direction_motor (TalonFX)
    tuner = tune_swerve_module(
        motor=swerve_module.direction_motor,
        name=f"Swerve Module {swerve_module.direction_motor.device_id}",
    )

    # Start tuning (call once)
    tuner.start_tuning(use_analytical=True, use_trial=True)

    # In your periodic loop, call:
    # if not tuner.is_complete():
    #     tuner.periodic()
    # else:
    #     gains = tuner.get_final_gains()
    #     print(f"Tuned gains: {gains}")

    return tuner


def example_tune_swerve_module_custom(swerve_module):
    """
    Advanced example: Tune swerve module with custom position getter.
    Useful when swerve module has complex gearing or uses a custom position method.
    """
    tuner = GenericMotorTuner(
        motor=swerve_module.direction_motor,
        control_type=ControlType.POSITION,
        name="Swerve Module FL",
        position_getter=lambda: swerve_module.getPosition().angle.radians(),
    )

    # Start both analytical and trial tuning
    tuner.start_tuning(use_analytical=True, use_trial=True)

    # In your periodic loop:
    # if not tuner.is_complete():
    #     tuner.periodic()
    # else:
    #     gains = tuner.get_final_gains()
    #     tuner.export_gains_to_file(f"swerve_module_{swerve_module.direction_motor.device_id}_gains.json")

    return tuner


def example_tune_flywheel_simple(shooter):
    """
    Simple example: Create a flywheel tuner.
    """
    ff_gains = FeedforwardGains(kS=0.2, kV=0.12, kA=0.01)
    tuner = tune_flywheel(
        motor=shooter.flywheel_motor,
        name="Shooter Flywheel",
        feedforward_gains=ff_gains,
    )
    tuner.start_tuning()

    # In periodic: tuner.periodic() until tuner.is_complete()
    # Then: gains = tuner.get_final_gains()

    return tuner


def example_tune_flywheel_analytical_only(shooter):
    """
    Tune flywheel using only analytical methods (faster, but potentially less accurate).
    """
    tuner = GenericMotorTuner(
        motor=shooter.flywheel_motor,
        control_type=ControlType.VELOCITY,
        name="Shooter Flywheel",
    )

    # Only analytical tuning
    tuner.start_tuning(use_analytical=True, use_trial=False, timeout_seconds=60.0)

    # In periodic: tuner.periodic() until tuner.is_complete()

    return tuner


def example_tune_dual_flywheels(shooter):
    """
    Example: Tune two flywheels (top and bottom) sequentially.
    You'll need to manage running them one at a time in your periodic loop.
    """
    # Create tuners for both flywheels
    top_tuner = tune_flywheel(motor=shooter.top_flywheel_motor, name="Top Flywheel")
    bottom_tuner = tune_flywheel(
        motor=shooter.bottom_flywheel_motor, name="Bottom Flywheel"
    )

    # Start first tuner (in your code, start second after first completes)
    top_tuner.start_tuning()

    return top_tuner, bottom_tuner


def example_tune_hood(shooter):
    """
    Tune a shooter hood (position control, analytical only recommended).
    """
    tuner = tune_hood(motor=shooter.hood_motor, name="Shooter Hood")
    tuner.start_tuning(use_trial=False)  # Analytical only

    return tuner


def example_tune_hood_with_custom_units(shooter):
    """
    Tune hood with custom position getter (e.g., returning angle in degrees).
    """
    import math

    tuner = GenericMotorTuner(
        motor=shooter.hood_motor,
        control_type=ControlType.POSITION,
        name="Shooter Hood",
        position_getter=lambda: shooter.get_hood_angle_degrees() * math.pi / 180.0,
    )

    tuner.start_tuning(use_analytical=True, use_trial=False)
    return tuner


def example_tune_elevator(elevator):
    """
    Tune an elevator with constant gravity compensation.
    """
    tuner = tune_elevator(
        motor=elevator.motor,
        name="Elevator",
        position_getter=lambda: elevator.get_height(),  # Returns height in meters
    )
    tuner.start_tuning()

    # After tuner.is_complete():
    # gains = tuner.get_final_gains()
    # print(f"Elevator gains: {gains}")

    return tuner


def example_tune_elevator_sparkmax(elevator):
    """
    Tune an elevator using a SparkMax motor controller.
    """
    tuner = GenericMotorTuner(
        motor=elevator.spark_max_motor,
        control_type=ControlType.POSITION,
        gravity_type=GravityType.CONSTANT,
        name="Elevator (SparkMax)",
        conversion_factor=0.1,  # Convert encoder units to meters
        position_getter=lambda: elevator.get_height_meters(),
    )

    tuner.start_tuning(use_analytical=True, use_trial=True)
    return tuner


def example_tune_arm_pivot(arm):
    """
    Tune an arm pivot with cosine gravity compensation.
    """
    tuner = tune_arm(
        motor=arm.pivot_motor,
        name="Arm Pivot",
        position_getter=lambda: arm.get_angle(),  # Returns angle in radians
    )
    tuner.start_tuning()

    # After tuner.is_complete():
    # gains = tuner.get_final_gains()
    # print(f"Arm gains: {gains}")

    return tuner


def example_tune_arm_with_prep(arm):
    """
    Tune arm with manual preparation step.

    Important: For cosine gravity compensation, the arm should be
    positioned horizontally before tuning begins.

    Note: The preparation should be done in your robot code before calling start_tuning().
    """
    # Move arm to horizontal position first (do this in your robot periodic before starting tuner)
    # print("Moving arm to horizontal position for gravity measurement...")
    # arm.move_to_horizontal()  # Your robot-specific method
    # Wait for arm to settle (in your periodic loop, not blocking)

    # Create and start tuner
    tuner = tune_arm(
        motor=arm.pivot_motor,
        name="Arm Pivot",
        position_getter=lambda: arm.get_angle_radians(),
    )
    tuner.start_tuning()

    return tuner


def example_tune_custom_mechanism(mechanism):
    """
    Tune a custom mechanism with full control over all parameters.
    """
    tuner = GenericMotorTuner(
        motor=mechanism.motor,
        control_type=ControlType.POSITION,  # or ControlType.VELOCITY
        gravity_type=GravityType.NONE,  # or CONSTANT, or COSINE
        name="Custom Mechanism",
        position_getter=lambda: mechanism.get_custom_position(),
        conversion_factor=1.0,  # Only used for SparkMax
    )

    # Start tuning with custom timeout
    tuner.start_tuning(
        use_analytical=True,
        use_trial=True,
        timeout_seconds=120.0,  # 2 minute timeout
    )

    # After tuner.is_complete():
    # results = tuner.get_results()
    # if results:
    #     print(f"\nDetailed Results for {mechanism}:")
    #     print(f"  Time Constant: {results.time_constant_pos:.3f} s")
    #     results.print_comparison()

    return tuner


def example_batch_tune_all_swerve_modules(drivetrain):
    """
    Create tuners for all four swerve modules.
    In your robot code, you'll need to run them sequentially in your periodic loop.
    """
    modules = [
        (drivetrain.front_left, "Front Left"),
        (drivetrain.front_right, "Front Right"),
        (drivetrain.rear_left, "Rear Left"),
        (drivetrain.rear_right, "Rear Right"),
    ]

    all_tuners = {}

    for module, name in modules:
        print(f"Creating tuner for {name} swerve module...")
        tuner = tune_swerve_module(
            motor=module.direction_motor,
            name=f"Swerve {name}",
            position_getter=lambda m=module: m.getPosition().angle.radians(),
        )
        all_tuners[name] = tuner

    # In your robot code, start and run these sequentially:
    # 1. Start first tuner: all_tuners["Front Left"].start_tuning()
    # 2. In periodic: if not tuner.is_complete(): tuner.periodic()
    # 3. When complete, start next tuner
    # 4. Repeat for all modules

    return all_tuners


def example_tune_complete_robot(robot):
    """
    Create tuners for all mechanisms on a robot.
    Note: These must be run sequentially in your robot's periodic loop.
    """
    tuners = {}

    # Create drivetrain tuners
    print("### CREATING DRIVETRAIN TUNERS ###")
    tuners["swerve"] = example_batch_tune_all_swerve_modules(robot.drivetrain)

    # Create shooter tuners
    print("### CREATING SHOOTER TUNERS ###")
    tuners["flywheel_top"] = tune_flywheel(robot.shooter.top_motor, "Top Flywheel")
    tuners["flywheel_bottom"] = tune_flywheel(
        robot.shooter.bottom_motor, "Bottom Flywheel"
    )
    tuners["hood"] = tune_hood(robot.shooter.hood_motor, "Hood")

    # Create elevator tuner
    print("### CREATING ELEVATOR TUNER ###")
    tuners["elevator"] = tune_elevator(robot.elevator.motor, "Elevator")

    # Create arm tuner
    print("### CREATING ARM TUNER ###")
    tuners["arm"] = tune_arm(robot.arm.pivot_motor, "Arm Pivot")

    print("\n" + "=" * 70)
    print("ALL TUNERS CREATED - Start and run them sequentially in your periodic loop")
    print("=" * 70)

    return tuners


def example_apply_saved_gains(
    motor,
    gains: MotorGains,
    feedforward_gains: FeedforwardGains | None = None,
):
    """
    Apply previously tuned gains to a motor.
    """
    tuner = GenericMotorTuner(
        motor=motor,
        control_type=ControlType.POSITION,
        name="Mechanism",  # Or VELOCITY
        feedforward_gains=feedforward_gains,
    )

    success = tuner.apply_gains(gains)

    if success:
        print("Gains applied successfully!")
    else:
        print("Failed to apply gains")

    return success


def example_load_and_apply_gains(motor, filename: str):
    """
    Load gains from a JSON file and apply them.
    """
    import json

    with open(filename, "r") as f:
        data = json.load(f)

    gains = MotorGains(
        kP=data["final_gains"]["kP"],
        kI=data["final_gains"]["kI"],
        kD=data["final_gains"]["kD"],
    )

    ff_data = data.get("feedforward_gains")
    ff_gains = None
    if ff_data:
        ff_gains = FeedforwardGains(
            kS=ff_data.get("kS", 0.0),
            kV=ff_data.get("kV", 0.0),
            kA=ff_data.get("kA", 0.0),
            kG=ff_data.get("kG", 0.0),
        )

    return example_apply_saved_gains(motor, gains, ff_gains)


def example_incremental_tuning(mechanism):
    """
    Start with analytical tuning, then refine with trial-and-error.
    """
    # First pass: Analytical only (quick)
    print("Phase 1: Analytical tuning (fast)...")
    tuner = GenericMotorTuner(
        motor=mechanism.motor, control_type=ControlType.POSITION, name="Mechanism"
    )

    tuner.start_tuning(use_analytical=True, use_trial=False, timeout_seconds=60.0)

    # In your periodic loop, run until complete:
    # while not tuner.is_complete():
    #     tuner.periodic()
    # analytical_gains = tuner.get_final_gains()
    # print("Test the robot with analytical gains. If not satisfied, run trial tuning.")

    # Second pass: Trial-and-error refinement (create new tuner or reset state)
    # For trial-only refinement, you would create a new tuner and start with use_analytical=False

    return tuner


def example_mixed_controllers(robot):
    """
    Create tuners for mechanisms with different motor controllers.
    """
    # TalonFX swerve modules
    swerve_tuner = tune_swerve_module(
        motor=robot.drivetrain.front_left.direction_motor,
        name="Swerve FL",  # TalonFX
    )

    # SparkMax elevator
    elevator_tuner = GenericMotorTuner(
        motor=robot.elevator.spark_motor,  # SparkMax
        control_type=ControlType.POSITION,
        gravity_type=GravityType.CONSTANT,
        name="Elevator (SparkMax)",
        conversion_factor=0.1,  # Encoder units to meters
    )

    # TalonFX arm
    arm_tuner = tune_arm(motor=robot.arm.talon_motor, name="Arm Pivot")  # TalonFX

    # Start them sequentially in your periodic loop
    return {"swerve": swerve_tuner, "elevator": elevator_tuner, "arm": arm_tuner}


def integrate_tuner_into_robot():
    """
    Example of how to integrate tuner into your robot.py using the non-blocking API.
    """

    example_robot_py = '''
from lemonlib.autopid.generic_motor_tuner import tune_swerve_module, tune_elevator, tune_arm
from magicbot import MagicRobot

class MyRobot(MagicRobot):
    # ... your robot components ...
    
    def createObjects(self):
        # ... create motors, etc ...
        self.active_tuner = None
        self.tuning_queue = []
    
    def teleopPeriodic(self):
        """Run tuning state machine in periodic loop"""
        
        # Start tuning when button pressed
        if self.joystick.getBackButtonPressed() and self.active_tuner is None:
            print("Starting auto-tune sequence...")
            self._setup_tuning_queue()
            self._start_next_tuner()
        
        # Run active tuner
        if self.active_tuner is not None:
            if not self.active_tuner.is_complete():
                self.active_tuner.periodic()
            else:
                # Current tuner complete
                gains = self.active_tuner.get_final_gains()
                print(f"Tuning complete! Gains: {gains}")
                
                # Start next tuner or finish
                self._start_next_tuner()
    
    def _setup_tuning_queue(self):
        """Create all tuners and add to queue"""
        self.tuning_queue = []
        
        # Swerve modules
        for module in [self.front_left, self.front_right, self.rear_left, self.rear_right]:
            tuner = tune_swerve_module(
                motor=module.direction_motor,
                name=f"Swerve {module.direction_motor.device_id}"
            )
            self.tuning_queue.append(tuner)
        
        # Elevator
        elevator_tuner = tune_elevator(motor=self.elevator.motor, name="Elevator")
        self.tuning_queue.append(elevator_tuner)
        
        # Arm
        arm_tuner = tune_arm(motor=self.arm.pivot_motor, name="Arm")
        self.tuning_queue.append(arm_tuner)
    
    def _start_next_tuner(self):
        """Start the next tuner in the queue"""
        if self.tuning_queue:
            self.active_tuner = self.tuning_queue.pop(0)
            self.active_tuner.start_tuning()
            print(f"Starting tuner: {self.active_tuner.name}")
        else:
            self.active_tuner = None
            print("Auto-tune sequence complete!")
'''

    print(example_robot_py)


if __name__ == "__main__":
    print(__doc__)
    print(
        "\nThis file contains comprehensive examples for using the Generic Motor Tuner."
    )
    print("Import the functions you need in your robot code.")
