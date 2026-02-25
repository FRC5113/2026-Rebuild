from commands2 import CommandScheduler, TimedCommandRobot
from sysidroutinebot import SysIdRoutineBot
from phoenix6 import SignalLogger


class MyRobot(TimedCommandRobot):
    def robotInit(self) -> None:
        self.robot = SysIdRoutineBot()

        self.robot.configureBindings()

    def teleopInit(self) -> None:
        CommandScheduler.getInstance().cancelAll()
