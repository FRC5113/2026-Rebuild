import math
import threading

import wpilib
from wpilib import TimedRobot, SmartDashboard, RobotController, Notifier

from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.configs import (
    TalonFXConfiguration,
    FeedbackConfigs,
    ClosedLoopGeneralConfigs,
)
from phoenix6.signals import FeedbackSensorSourceValue, NeutralModeValue
from phoenix6 import controls
from phoenix6 import CANBus


class MyRobot(TimedRobot):

    def robotInit(self):
        # --------------------
        # Constants
        # --------------------
        self.steer_gear_ratio = 150.0 / 7.0
        self.target_angle = math.pi / 2  # 90 deg

        # --------------------
        # Hardware
        # --------------------
        self.canbus = CANBus("can0")
        self.steer_motor = TalonFX(42, self.canbus)
        self.cancoder = CANcoder(43, self.canbus)

        # --------------------
        # Motor configuration
        # --------------------
        motor_cfg = TalonFXConfiguration()
        motor_cfg.motor_output.neutral_mode = NeutralModeValue.BRAKE

        motor_cfg.feedback = (
            FeedbackConfigs()
            .with_feedback_remote_sensor_id(self.cancoder.device_id)
            .with_feedback_sensor_source(
                FeedbackSensorSourceValue.FUSED_CANCODER
            )
            .with_rotor_to_sensor_ratio(self.steer_gear_ratio)
        )

        motor_cfg.closed_loop_general = (
            ClosedLoopGeneralConfigs().with_continuous_wrap(True)
        )

        self.steer_motor.configurator.apply(motor_cfg)

        self.control = (
            controls.PositionVoltage(0)
            .with_slot(0)
            .with_enable_foc(True)
        )

        # --------------------
        # Shared state
        # --------------------
        self.lock = threading.Lock()
        self.current_angle = 0.0
        self.error = 0.0

        # --------------------
        # Tuning state
        # --------------------
        self.stage = "p_sweep"

        self.p_test = 20.0
        self.p_step = 2.0
        self.p_osc = None

        self.d_test = 0.0
        self.d_step = 0.05

        self.crossings = 0
        self.last_sign = 0

        # --------------------
        # Start notifier
        # --------------------
        self.tune_notifier = Notifier(self._tune_loop)
        self.tune_notifier.startPeriodic(0.25)  # 250 ms

        print("Notifier-based PID tuning started")

    # --------------------------------------------------

    def teleopPeriodic(self):
        # Fast, deterministic loop ONLY
        self.steer_motor.set_control(
            self.control.with_position(self.target_angle / math.tau)
        )

        angle = self.cancoder.get_absolute_position().value * math.tau
        error = (self.target_angle - angle + math.pi) % (2 * math.pi) - math.pi

        sign = 1 if error > 0 else -1
        if sign != self.last_sign:
            self.crossings += 1
        self.last_sign = sign

        with self.lock:
            self.current_angle = angle
            self.error = error

        SmartDashboard.putString("Stage", self.stage)
        SmartDashboard.putNumber("Error (deg)", error * 180 / math.pi)

    # --------------------------------------------------

    def _tune_loop(self):
        # Runs in Notifier thread (SLOW + SAFE)
        with self.lock:
            error = self.error
            crossings = self.crossings
            self.crossings = 0

        if self.stage == "p_sweep":
            if crossings >= 3:
                self.p_osc = self.p_test
                self.p_test *= 0.7
                self._apply_gains(self.p_test, 0.0)
                print(f"P oscillation at {self.p_osc:.2f}, using {self.p_test:.2f}")
                self.stage = "d_sweep"
            else:
                self.p_test += self.p_step
                self._apply_gains(self.p_test, 0.0)
                print(f"Testing P = {self.p_test:.2f}")

        elif self.stage == "d_sweep":
            overshoot_deg = abs(error) * 180 / math.pi

            if overshoot_deg < 2.0:
                print("Tuning complete")
                print(f"FINAL kP = {self.p_test:.2f}")
                print(f"FINAL kD = {self.d_test:.2f}")
                self.stage = "done"
                self.tune_notifier.stop()
            else:
                self.d_test += self.d_step
                self._apply_gains(self.p_test, self.d_test)
                print(f"Testing D = {self.d_test:.2f}")

    # --------------------------------------------------

    def _apply_gains(self, p, d):
        cfg = TalonFXConfiguration()
        cfg.slot0.k_p = p
        cfg.slot0.k_i = 0.0
        cfg.slot0.k_d = d
        self.steer_motor.configurator.apply(cfg)

    # --------------------------------------------------

    def disabledInit(self):
        if hasattr(self, "tune_notifier"):
            self.tune_notifier.stop()


if __name__ == "__main__":
    wpilib.run(MyRobot)
