import math
from phoenix6 import unmanaged
from phoenix6.hardware.talon_fx import TalonFX
from rev import SparkMaxSim, SparkRelativeEncoderSim, SparkMax, SparkAbsoluteEncoderSim
from pyfrc.physics.core import PhysicsInterface
from pyfrc.physics.drivetrains import four_motor_swerve_drivetrain

# from photonlibpy.simulation.photonCameraSim import PhotonCameraSim
# from photonlibpy.simulation.simCameraProperties import SimCameraProperties
# from photonlibpy.simulation.visionSystemSim import VisionSystemSim
from wpilib import DriverStation, Mechanism2d, SmartDashboard, Color8Bit
from wpilib.simulation import SingleJointedArmSim, ElevatorSim, DIOSim
from wpimath.system.plant import DCMotor, LinearSystemId
from wpimath.geometry import Pose2d, Transform2d, Rotation3d
from robot import MyRobot

# from lemonlib.simulation import LemonCameraSim
from lemonlib.simulation import FalconSim, FalconSimFOC, KrakenSim, KrakenSimFOC


class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: MyRobot):
        # Swerve Drive Setup
        self.physics_controller = physics_controller
        self.robot = robot
        self.pose = Pose2d()
        self.speed_sims = (
            KrakenSimFOC(robot.front_left_speed_motor, 0.01, 6.75),
            KrakenSimFOC(robot.front_right_speed_motor, 0.01, 6.75),
            KrakenSimFOC(robot.rear_left_speed_motor, 0.01, 6.75),
            KrakenSimFOC(robot.rear_right_speed_motor, 0.01, 6.75),
        )
        self.direction_sims = (
            KrakenSimFOC(robot.front_left_direction_motor, 0.01, 150 / 7),
            KrakenSimFOC(robot.front_right_direction_motor, 0.01, 150 / 7),
            KrakenSimFOC(robot.rear_left_direction_motor, 0.01, 150 / 7),
            KrakenSimFOC(robot.rear_right_direction_motor, 0.01, 150 / 7),
        )

        self.encoders = (
            robot.front_left_cancoder,
            robot.front_right_cancoder,
            robot.rear_left_cancoder,
            robot.rear_right_cancoder,
        )
        for encoder in self.encoders:
            encoder.sim_state.add_position(0.25)

        self.robot.pigeon.sim_state.set_supply_voltage(5.0)

    def update_sim(self, now, tm_diff):
        if DriverStation.isEnabled():
            unmanaged.feed_enable(100)

            if self.robot.swerve_drive.starting_pose is not None:
                self.physics_controller.move_robot(
                    Transform2d(self.pose.translation(), self.pose.rotation()).inverse()
                )
                start = self.robot.swerve_drive.starting_pose
                self.physics_controller.move_robot(
                    Transform2d(start.translation(), start.rotation())
                )
                self.robot.swerve_drive.set_starting_pose(None)

            for i in range(4):
                self.speed_sims[i].update(tm_diff)
                self.direction_sims[i].update(tm_diff)
                self.encoders[i].sim_state.add_position(
                    -self.direction_sims[i].motor_sim.getAngularVelocity()
                    / (2 * math.pi)
                    * tm_diff
                )

            sim_speeds = four_motor_swerve_drivetrain(
                self.speed_sims[2].sim_state.motor_voltage / 12.0,
                self.speed_sims[3].sim_state.motor_voltage / 12.0,
                self.speed_sims[0].sim_state.motor_voltage / 12.0,
                self.speed_sims[1].sim_state.motor_voltage / 12.0,
                (self.encoders[2].get_absolute_position().value * -360) % 360,
                (self.encoders[3].get_absolute_position().value * -360) % 360,
                (self.encoders[0].get_absolute_position().value * -360) % 360,
                (self.encoders[1].get_absolute_position().value * -360) % 360,
                2.5,
                2.5,
                15.52,
            )
            # Artificially soften simulated omega
            sim_speeds.omega_dps *= 0.4
            # Correct chassis speeds to match initial robot orientation
            sim_speeds.vx, sim_speeds.vy = sim_speeds.vy, -sim_speeds.vx
            self.pose = self.physics_controller.drive(sim_speeds, tm_diff)
            # self.robot.camera.set_robot_pose(pose)
            self.robot.pigeon.sim_state.set_raw_yaw(
                self.pose.rotation().degrees() + 180
            )
