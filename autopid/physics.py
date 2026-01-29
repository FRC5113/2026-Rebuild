#
# See the documentation for more details on how this works
#
# Documentation can be found at https://robotpy.readthedocs.io/projects/pyfrc/en/latest/physics.html
#
# The idea here is you provide a simulation object that overrides specific
# pieces of WPILib, and modifies motors/sensors accordingly depending on the
# state of the simulation. An example of this would be measuring a motor
# moving for a set period of time, and then changing a limit switch to turn
# on after that period of time. This can help you do more complex simulations
# of your robot code without too much extra effort.
#
# Examples can be found at https://github.com/robotpy/examples

import math
from phoenix6 import unmanaged
from phoenix6.hardware.talon_fx import TalonFX
from pyfrc.physics.core import PhysicsInterface
from pyfrc.physics.drivetrains import four_motor_swerve_drivetrain
from wpilib.simulation import DCMotorSim
from wpimath.geometry import Pose2d, Transform2d
from wpimath.system.plant import DCMotor, LinearSystemId


import typing

if typing.TYPE_CHECKING:
    from robot import MyRobot


class KrakenSimFOC:
    def __init__(self, motor: TalonFX, moi: float, gearing: float):
        self.gearbox = DCMotor.krakenX60FOC(1)
        self.plant = LinearSystemId.DCMotorSystem(self.gearbox, moi, gearing)
        self.gearing = gearing
        self.sim_state = motor.sim_state
        self.sim_state.set_supply_voltage(12.0)
        self.motor_sim = DCMotorSim(self.plant, self.gearbox, [0.0001, 0.0001])

    def getSetpoint(self) -> float:
        return self.sim_state.motor_voltage

    def update(self, dt: float):
        voltage = self.sim_state.motor_voltage
        self.motor_sim.setInputVoltage(voltage)
        self.motor_sim.update(dt)
        self.sim_state.set_raw_rotor_position(
            self.motor_sim.getAngularPositionRotations() * self.gearing
        )
        self.sim_state.set_rotor_velocity(
            self.motor_sim.getAngularVelocityRPM() / 60 * self.gearing
        )


class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: "MyRobot"):
        # Swerve Drive Setup
        self.physics_controller = physics_controller
        self.robot = robot
        self.pose = Pose2d()

        # Create speed motor simulations
        self.speed_sims = (
            KrakenSimFOC(robot.front_left_speed_motor, 0.01, 6.75),
            KrakenSimFOC(robot.front_left_speed_motor, 0.01, 6.75),
            KrakenSimFOC(robot.front_left_speed_motor, 0.01, 6.75),
            KrakenSimFOC(robot.front_left_speed_motor, 0.01, 6.75),
        )

        # Create direction motor simulations
        self.direction_sims = (
            KrakenSimFOC(robot.front_left_speed_motor, 0.01, 150 / 7),
            KrakenSimFOC(robot.front_left_speed_motor, 0.01, 150 / 7),
            KrakenSimFOC(robot.front_left_speed_motor, 0.01, 150 / 7),
            KrakenSimFOC(robot.front_left_speed_motor, 0.01, 150 / 7),
        )

        # CANcoder references
        self.encoders = (
            robot.front_left_cancoder,
            robot.front_left_cancoder,
            robot.front_left_cancoder,
            robot.front_left_cancoder,
        )

        # Initialize encoder positions
        for encoder in self.encoders:
            encoder.sim_state.add_position(0.25)

    def update_sim(self, now, tm_diff):
        # Feed the enable signal to Phoenix
        unmanaged.feed_enable(100)

        # Update all motor simulations
        for i in range(4):
            self.speed_sims[i].update(tm_diff)
            self.direction_sims[i].update(tm_diff)

            # Update encoder position based on direction motor velocity
            self.encoders[i].sim_state.add_position(
                -self.direction_sims[i].motor_sim.getAngularVelocity()
                / (2 * math.pi)
                * tm_diff
            )

        # Calculate robot movement based on swerve module states
        sim_speeds = four_motor_swerve_drivetrain(
            self.speed_sims[2].sim_state.motor_voltage / 12.0,  # Back left
            self.speed_sims[3].sim_state.motor_voltage / 12.0,  # Back right
            self.speed_sims[0].sim_state.motor_voltage / 12.0,  # Front left
            self.speed_sims[1].sim_state.motor_voltage / 12.0,  # Front right
            (self.encoders[2].get_absolute_position().value * -360)
            % 360,  # Back left angle
            (self.encoders[3].get_absolute_position().value * -360)
            % 360,  # Back right angle
            (self.encoders[0].get_absolute_position().value * -360)
            % 360,  # Front left angle
            (self.encoders[1].get_absolute_position().value * -360)
            % 360,  # Front right angle
            2.25,  # x_wheelbase
            2.25,  # y_wheelbase
            15.52,  # speed
        )

        # Artificially soften simulated omega
        sim_speeds.omega_dps *= 0.4

        # Correct chassis speeds to match initial robot orientation
        sim_speeds.vx, sim_speeds.vy = sim_speeds.vy, sim_speeds.vx

        # Update robot pose
        self.pose = self.physics_controller.drive(sim_speeds, tm_diff)
