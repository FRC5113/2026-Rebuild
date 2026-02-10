from typing import Callable, List, Tuple

import hal
import magicbot
from pykit.logger import Logger
from wpilib import DriverStation, Notifier, RobotController


class LemonRobot(magicbot.MagicRobot):
    """
    Wrapper for the magicbot robot class to allow for command-based
    functionality. This class is used to create a robot that can be
    controlled using commands, while still using the magicbot framework.
    """

    low_bandwidth = DriverStation.isFMSAttached()

    def __init__(self):
        super().__init__()
        self._periodic_callbacks: List[Tuple[Callable[[], None], float]] = []
        self._notifiers: list[Notifier] = []

        self.loop_time = self.control_loop_wait_time
        print("LemonRobot initialized")

    def add_periodic(self, callback: Callable[[], None], period: float):
        print(f"Registering periodic: {callback.__name__}, every {period}s")
        self._periodic_callbacks.append((callback, period))

    def autonomousPeriodic(self):
        """
        Periodic code for autonomous mode should go here.
        Runs when not enabled for trajectory display.

        Users should override this method for code which will be called
        periodically at a regular rate while the robot is in autonomous mode.

        This code executes before the ``execute`` functions of all
        components are called.
        """
        pass

    def startCompetition(self) -> None:
        """
        This runs the mode-switching loop.

        .. warning:: Internal API, don't override
        """

        # TODO: usage reporting?
        self.robotInit()

        self.initEnd = RobotController.getFPGATime()
        Logger.periodicAfterUser(self.initEnd, 0)

        # Tell the DS the robot is ready to be enabled
        hal.observeUserProgramStarting()

        Logger.startReciever()

        while not self._MagicRobot__done:
            isEnabled, isAutonomous, isTest = self.getControlState()
            # Run logger pre-user code (load inputs from log or sensors)
            periodicBeforeStart = RobotController.getFPGATime()
            Logger.periodicBeforeUser()

            # Execute user periodic code and measure timing
            userCodeStart = RobotController.getFPGATime()
            if not isEnabled:
                self._disabled()
            elif isAutonomous:
                self.autonomous()
            elif isTest:
                self._test()
            else:
                self._operatorControl()
            userCodeEnd = RobotController.getFPGATime()

            # Run logger post-user code (save outputs to log)
            Logger.periodicAfterUser(
                userCodeEnd - userCodeStart, userCodeStart - periodicBeforeStart
            )

    # def endCompetition(self) -> None:
    #     self.__done = True
    #     if self._automodes:
    #         self._automodes.endCompetition()

    def autonomous(self):
        super().autonomous()
        self.autonomousPeriodic()

    def enabledperiodic(self) -> None:
        """Periodic code for when the bot is enabled should go here.
        Runs when not enabled for trajectory display.

        Users should override this method for code which will be called"""
        pass

    def _stop_notifiers(self):
        for notifier in self._notifiers:
            notifier.stop()
        self._notifiers.clear()

    def _on_mode_disable_components(self):
        super()._on_mode_disable_components()
        self._stop_notifiers()

    def _on_mode_enable_components(self):
        super()._on_mode_enable_components()
        self.on_enable()
        self._restart_periodics()

    def on_enable(self):
        pass

    def _restart_periodics(self):
        self._stop_notifiers()
        for callback, period in self._periodic_callbacks:
            notifier = Notifier(callback)
            notifier.setName(f"Periodic-{callback.__name__}")
            notifier.startPeriodic(period)
            self._notifiers.append(notifier)

    def _enabled_periodic(self) -> None:
        """Run components and all periodic methods."""
        watchdog = self.watchdog

        for name, component in self._components:
            try:
                component.execute()

            except Exception:
                self.onException()
            watchdog.addEpoch(name)

        self.enabledperiodic()

        self._do_periodics()

        for reset_dict, component in self._reset_components:
            component.__dict__.update(reset_dict)

    def _do_periodics(self):
        super()._do_periodics()

        self.loop_time = max(self.control_loop_wait_time, self.watchdog.getTime())

    def get_period(self) -> float:
        """Get the period of the robot loop in seconds."""
        return self.loop_time
