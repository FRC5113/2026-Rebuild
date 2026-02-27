from phoenix6 import CANBus, configs, signals, swerve, units


class TunerConstants:
    """
    Swerve drivetrain constants matching the existing SDS MK4i module configuration.
    """

    # The steer motor uses any SwerveModule.SteerRequestType control request
    # with the output type specified by SwerveModuleConstants.steer_motor_closed_loop_output
    steer_gains = (
        configs.Slot0Configs()
        .with_k_p(41.0)
        .with_k_i(0.0)
        .with_k_d(1.0)
        .with_k_s(0.23)
        .with_k_v(2.6)
        .with_k_a(0.11)
        .with_static_feedforward_sign(
            signals.StaticFeedforwardSignValue.USE_CLOSED_LOOP_SIGN
        )
    )

    # When using closed-loop control, the drive motor uses the control
    # output type specified by SwerveModuleConstants.drive_motor_closed_loop_output
    drive_gains = (
        configs.Slot0Configs()
        .with_k_p(0.0)
        .with_k_i(0.0)
        .with_k_d(0.0)
        .with_k_s(0.17)
        .with_k_v(0.104)
        .with_k_a(0.01)
    )

    # The closed-loop output type to use for the steer motors
    steer_closed_loop_output = swerve.ClosedLoopOutputType.VOLTAGE
    # The closed-loop output type to use for the drive motors
    drive_closed_loop_output = swerve.ClosedLoopOutputType.VOLTAGE

    # Motor types
    drive_motor_type = swerve.DriveMotorArrangement.TALON_FX_INTEGRATED
    steer_motor_type = swerve.SteerMotorArrangement.TALON_FX_INTEGRATED

    # Steer feedback type (Fused CANcoder for Pro-licensed devices)
    steer_feedback_type = swerve.SteerFeedbackType.FUSED_CANCODER

    # The stator current at which the wheels start to slip
    slip_current: units.ampere = 60.0

    # Initial configs for the drive and steer motors and the azimuth encoder
    drive_initial_configs = configs.TalonFXConfiguration()
    steer_initial_configs = configs.TalonFXConfiguration().with_current_limits(
        configs.CurrentLimitsConfigs()
        .with_stator_current_limit(40.0)
        .with_stator_current_limit_enable(True)
    )
    encoder_initial_configs = configs.CANcoderConfiguration()

    # Pigeon 2 configs; set to None to skip applying configs
    pigeon_configs: configs.Pigeon2Configuration | None = None

    # CAN bus
    canbus = CANBus("can0")

    # Theoretical free speed (m/s) at 12V applied output
    speed_at_12_volts: units.meters_per_second = 4.7

    # Coupling ratio: every 1 rotation of the azimuth results in this many drive motor turns
    # For SDS MK4i L2 this is approximately 3.5714 (50/14)
    couple_ratio = 50.0 / 14.0

    # Gear ratios
    drive_gear_ratio = 6.75  # SDS MK4i L2
    steer_gear_ratio = 150.0 / 7.0  # SDS MK4i

    # Wheel radius
    wheel_radius: units.meter = 0.0508  # 2 inch wheels

    # Module positions (distance from robot center to each wheel)
    offset_x: units.meter = 0.3429
    offset_y: units.meter = 0.3429

    # Invert sides: left side is not inverted, right side is inverted
    invert_left_side = False
    invert_right_side = True

    # Pigeon ID
    pigeon_id = 30

    # Simulation constants
    steer_inertia: units.kilogram_square_meter = 0.01
    drive_inertia: units.kilogram_square_meter = 0.01
    steer_friction_voltage: units.volt = 0.2
    drive_friction_voltage: units.volt = 0.2

    # Drivetrain Constants
    drivetrain_constants = (
        swerve.SwerveDrivetrainConstants()
        .with_can_bus_name(canbus.name)
        .with_pigeon2_id(pigeon_id)
        .with_pigeon2_configs(pigeon_configs)
    )

    # Module Constants Factory
    constants_creator: swerve.SwerveModuleConstantsFactory[
        configs.TalonFXConfiguration,
        configs.TalonFXConfiguration,
        configs.CANcoderConfiguration,
    ] = (
        swerve.SwerveModuleConstantsFactory()
        .with_drive_motor_gear_ratio(drive_gear_ratio)
        .with_steer_motor_gear_ratio(steer_gear_ratio)
        .with_coupling_gear_ratio(couple_ratio)
        .with_wheel_radius(wheel_radius)
        .with_steer_motor_gains(steer_gains)
        .with_drive_motor_gains(drive_gains)
        .with_steer_motor_closed_loop_output(steer_closed_loop_output)
        .with_drive_motor_closed_loop_output(drive_closed_loop_output)
        .with_slip_current(slip_current)
        .with_speed_at12_volts(speed_at_12_volts)
        .with_drive_motor_type(drive_motor_type)
        .with_steer_motor_type(steer_motor_type)
        .with_feedback_source(steer_feedback_type)
        .with_drive_motor_initial_configs(drive_initial_configs)
        .with_steer_motor_initial_configs(steer_initial_configs)
        .with_encoder_initial_configs(encoder_initial_configs)
        .with_steer_inertia(steer_inertia)
        .with_drive_inertia(drive_inertia)
        .with_steer_friction_voltage(steer_friction_voltage)
        .with_drive_friction_voltage(drive_friction_voltage)
    )

    # Module-specific constants

    # Front Left
    front_left_drive_motor_id = 11
    front_left_steer_motor_id = 12
    front_left_encoder_id = 13
    front_left_encoder_offset: units.rotation = 0.0
    front_left_steer_motor_inverted = True
    front_left_encoder_inverted = False

    # Front Right
    front_right_drive_motor_id = 21
    front_right_steer_motor_id = 22
    front_right_encoder_id = 23
    front_right_encoder_offset: units.rotation = 0.0
    front_right_steer_motor_inverted = True
    front_right_encoder_inverted = False

    # Back Left
    back_left_drive_motor_id = 41
    back_left_steer_motor_id = 42
    back_left_encoder_id = 43
    back_left_encoder_offset: units.rotation = 0.0
    back_left_steer_motor_inverted = True
    back_left_encoder_inverted = False

    # Back Right
    back_right_drive_motor_id = 31
    back_right_steer_motor_id = 32
    back_right_encoder_id = 33
    back_right_encoder_offset: units.rotation = 0.0
    back_right_steer_motor_inverted = True
    back_right_encoder_inverted = False

    # Create module constants
    front_left = constants_creator.create_module_constants(
        front_left_steer_motor_id,
        front_left_drive_motor_id,
        front_left_encoder_id,
        front_left_encoder_offset,
        offset_x,
        offset_y,
        invert_left_side,
        front_left_steer_motor_inverted,
        front_left_encoder_inverted,
    )
    front_right = constants_creator.create_module_constants(
        front_right_steer_motor_id,
        front_right_drive_motor_id,
        front_right_encoder_id,
        front_right_encoder_offset,
        offset_x,
        -offset_y,
        invert_right_side,
        front_right_steer_motor_inverted,
        front_right_encoder_inverted,
    )
    back_left = constants_creator.create_module_constants(
        back_left_steer_motor_id,
        back_left_drive_motor_id,
        back_left_encoder_id,
        back_left_encoder_offset,
        -offset_x,
        offset_y,
        invert_left_side,
        back_left_steer_motor_inverted,
        back_left_encoder_inverted,
    )
    back_right = constants_creator.create_module_constants(
        back_right_steer_motor_id,
        back_right_drive_motor_id,
        back_right_encoder_id,
        back_right_encoder_offset,
        -offset_x,
        -offset_y,
        invert_right_side,
        back_right_steer_motor_inverted,
        back_right_encoder_inverted,
    )
