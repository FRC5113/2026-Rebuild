from commands2.sysid import SysIdRoutine
from phoenix6 import SignalLogger
from wpilib.sysid import SysIdRoutineLog
from wpimath import units

from components.drive_control import DriveControl
from components.swerve_drive import SwerveDrive
from lemonlib.util import MagicSysIdRoutine


class SysIdDriveLinear(MagicSysIdRoutine):
    drive_control: DriveControl
    swerve_drive: SwerveDrive
    period: units.seconds = 0.02

    def setup(self):
        self.setup_sysid(
            SysIdRoutine.Config(
                rampRate=1,
                stepVoltage=7.0,
                recordState=lambda state: SignalLogger.write_string(
                    "state", SysIdRoutineLog.stateEnumToString(state)
                ),
            ),
            SysIdRoutine.Mechanism(
                self.drive_sysid,
                self.swerve_drive.log,
                self.swerve_drive,
                "Drive Linear",
            ),
        )

    def drive_sysid(self, voltage: units.volts) -> None:
        self.drive_control.drive_sysid_manual(voltage)


class SysIdDriveRotation(MagicSysIdRoutine):
    drive_control: DriveControl
    swerve_drive: SwerveDrive
    period: units.seconds = 0.02

    def setup(self):
        self.setup_sysid(
            SysIdRoutine.Config(rampRate=0.2, stepVoltage=7.0),
            SysIdRoutine.Mechanism(
                self.drive_sysid,
                self.swerve_drive.log,
                self.swerve_drive,
                "Drive Rotatinal",
            ),
        )

    def drive_sysid(self, voltage: units.volts) -> None:
        self.drive_control.drive_sysid_manual(voltage, 1)
