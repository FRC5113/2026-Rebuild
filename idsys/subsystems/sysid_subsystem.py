from commands2 import Command, Subsystem
from commands2.sysid import SysIdRoutine
from phoenix6 import SignalLogger
from wpilib import sysid
from wpimath.units import volts


class SysidSubsystem(Subsystem):
    logger_inited = False

    def __init__(
        self,
    ) -> None:

        self.sys_id_routine = SysIdRoutine(
            SysIdRoutine.Config(recordState=self.recordState),
            SysIdRoutine.Mechanism(self.drive, self.log, self),
        )

    def drive(self, voltage: volts) -> None:
        raise NotImplementedError(f"{type(self).__qualname__}.drive() not implemented")

    def log(self, sys_id_routine: sysid.SysIdRoutineLog) -> None:
        raise NotImplementedError(f"{type(self).__qualname__}.log() not implemented")

    def recordState(self, state: sysid.State) -> None:
        if not SysidSubsystem.logger_inited:
            SignalLogger.start()
            SysidSubsystem.logger_inited = True

        SignalLogger.write_string(
            f"sysid-test-state-{self.getName()}",
            sysid.SysIdRoutineLog.stateEnumToString(state),
        )
        self.sys_id_routine.recordState(state)

    def defaultCommand(self) -> Command:
        return self.run(lambda: None)

    def sysIdQuasistatic(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine.quasistatic(direction)

    def sysIdDynamic(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine.dynamic(direction)