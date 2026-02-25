import phoenix6
from commands2.button import CommandXboxController, Trigger
from commands2.sysid import SysIdRoutine
from subsystems.arm import Arm
from subsystems.flywheel import Flywheel
from subsystems.swerve_module import SwerveModule
from subsystems.sysid_subsystem import SysidSubsystem


class SysIdRoutineBot:
    def __init__(self) -> None:

        # This is only for the drive motors
        # Make sure to have configured the proper gear reduction and gains internally
        # self.swerve_drive = SwerveDrive()
        self.swerve_module = SwerveModule()

        # This can be applied to general flywheel systems
        #  - shooter
        # if using a swerve steer make sure to comment out references to the drive
        # system to avoid double motor initialisation
        self.flywheel = Flywheel(
            phoenix6.hardware.TalonFX(3),
            (phoenix6.hardware.TalonFX(2), True),
            gearing=1.0,
            name="flywheel",
        )

        # Single-jointed arm – adjust CAN ID, gearing, and soft limits to match your mechanism
        self.arm = Arm(
            phoenix6.hardware.TalonFX(4),
            gearing=100.0,
            name="arm",
        )

        self.controller = CommandXboxController(0)

    def configureBindings(self) -> None:
        # self.swerve_drive.setDefaultCommand(self.swerve_drive.defaultCommand())
        self.flywheel.setDefaultCommand(self.flywheel.defaultCommand())
        self.swerve_module.setDefaultCommand(self.swerve_module.defaultCommand())
        self.arm.setDefaultCommand(self.arm.defaultCommand())

        def bindSysId(subsystem: SysidSubsystem, pov: Trigger):
            (pov & self.controller.a()).whileTrue(
                subsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
            )
            (pov & self.controller.b()).whileTrue(
                subsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
            )
            (pov & self.controller.x()).whileTrue(
                subsystem.sysIdDynamic(SysIdRoutine.Direction.kForward)
            )
            (pov & self.controller.y()).whileTrue(
                subsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse)
            )

        # bindSysId(self.swerve_drive, self.controller.povUp())
        bindSysId(self.flywheel, self.controller.povLeft())
        bindSysId(self.swerve_module, self.controller.povDown())
        bindSysId(self.arm, self.controller.povRight())
