from commands2 import CommandScheduler, TimedCommandRobot

from sysidroutinebot import SysIdRoutineBot


class MyRobot(TimedCommandRobot):

    def robotInit(self) -> None:
        self.robot = SysIdRoutineBot()

        self.robot.configureBindings()

    def teleopInit(self) -> None:
        CommandScheduler.getInstance().cancelAll()