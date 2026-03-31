package frc.robot;

/**
 * Robot-wide constants: CAN IDs, current limits, voltages, and operator
 * scaling.
 * See README.md for hardware mapping and behavior.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int LEFT_LEADER_ID = 11;
    public static final int LEFT_FOLLOWER_ID = 8;
    public static final int RIGHT_LEADER_ID = 10;
    public static final int RIGHT_FOLLOWER_ID = 7;

    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;
    /** Wheel diameter in meters (e.g. 6 in ≈ 0.1524 m). */
    public static final double WHEEL_DIAMETER_METERS = 0.1524;
    /** Gear ratio motor-to-wheel (e.g. 10.71 for KitBot). */
    public static final double GEAR_RATIO = 10.71;

    /**
     * {@code true} when quadrature encoders are wired to the drive Spark MAX data
     * ports (required for brushed CIM odometry). Brushless Neos use internal
     * encoders; without external encoders on CIM drive, distance/velocity
     * readings are not meaningful — set {@code false}.
     */
    public static final boolean DRIVE_QUADRATURE_ENCODERS_WIRED = false;

    /** How fast the intake wiggles back and forth to unjam balls. */
    public static final double INTAKE_WIGGLE_SPEED = 0.46;
  }

  public static final class IoConstants {
    public static final int IO_MOTOR_ID = 9;
    /**
     * Intake (12) – pulls fuel from floor/storage. Anti-clockwise = intake;
     * clockwise = spit out.
     */
    public static final int INTAKE_MOTOR_ID = 12;
    public static final int LOADER_MOTOR_ID = 19;
    /** Current limits (amps) for IO / intake / loader motors. */
    public static final int IO_MOTOR_CURRENT_LIMIT = 60;
    public static final int INTAKE_MOTOR_CURRENT_LIMIT = 60;
    public static final int LOADER_MOTOR_CURRENT_LIMIT = 60;
    /** Current limit (amps) for the shooter flywheel motor. */
    public static final int FLYWHEEL_MOTOR_CURRENT_LIMIT = 60;

    // -----------------------------------------------------------------------
    // Shooter / intake speed control (encoder-based)
    // -----------------------------------------------------------------------

    /** Target shooter speed (RPM) for the main shooting/intake command. */
    public static final double SHOOTER_TARGET_SPEED_INTAKE_RPM = 2000.0;
    /** Target shooter speed (RPM) for the 50% spin-up toggle. */
    public static final double SHOOTER_TARGET_SPEED_SPINUP50_RPM = 2000.0;
    /** Target shooter speed (RPM) when using the right-trigger toggle. */
    public static final double SHOOTER_TARGET_SPEED_TOGGLE_RPM = SHOOTER_TARGET_SPEED_INTAKE_RPM;
    /** Target shooter speed (RPM) for the main launch shot. */
    public static final double SHOOTER_TARGET_SPEED_LAUNCH_RPM = 2000.0;
    /** Target shooter speed (RPM) for a high-speed shot (A button). */
    public static final double SHOOTER_TARGET_SPEED_HIGH_RPM = 1900.0;
    /** Target shooter speed (RPM) for an ultra-speed shot (long range). */
    public static final double SHOOTER_TARGET_SPEED_ULTRA_RPM = 6000.0;

    /**
     * Fraction of target speed below which we apply max voltage to spin up quickly.
     * For example, 0.8 means full voltage until 80% of target speed is reached.
     */
    public static final double SHOOTER_SPINUP_THRESHOLD_FRACTION = 0.8;

    /** Maximum voltage the shooter is ever commanded to (absolute value). */
    public static final double SHOOTER_MAX_VOLTAGE = 12.0;

    // --- Shooter velocity PID + feedforward (see docs/IO_PID.md) ---
    /** kP: output volts per (rot/s) error. */
    public static final double SHOOTER_PID_KP = 0.12;
    /** kI: integral term on velocity error. */
    public static final double SHOOTER_PID_KI = 0.25;
    /**
     * kD: derivative on velocity (often small; encoder velocity can be noisy).
     */
    public static final double SHOOTER_PID_KD = 0.0001;
    /** Static friction feedforward (volts). */
    public static final double SHOOTER_FF_KS = 0.0;
    /** Velocity feedforward (V·s/rotation). */
    public static final double SHOOTER_FF_KV = 0.30;
    /** Acceleration feedforward (V·s²/rotation); 0 if unknown. */
    public static final double SHOOTER_FF_KA = 0.0;
    /** Absolute limit for PID integrator accumulator (WPILib units). */
    public static final double SHOOTER_PID_INTEGRATOR_MAX = 2.0;

    // -----------------------------------------------------------------------
    // Intake speed control (encoder-based PID + feedforward)
    // -----------------------------------------------------------------------

    /** Target intake speed (RPM) for intake/feed behavior. Negative = intake in. */
    public static final double INTAKE_TARGET_SPEED_RPM = -2500.0;
    /** Spin-up threshold fraction before switching to closed-loop control. */
    public static final double INTAKE_SPINUP_THRESHOLD_FRACTION = 0.8;
    /** Absolute maximum intake voltage command. */
    public static final double INTAKE_MAX_VOLTAGE = 11.5;

    public static final double INTAKE_PID_KP = 0.15;
    public static final double INTAKE_PID_KI = 0.35;
    public static final double INTAKE_PID_KD = 0.0001;
    public static final double INTAKE_FF_KS = 0.0;
    public static final double INTAKE_FF_KV = 0.17;
    public static final double INTAKE_FF_KA = 0.0;
    public static final double INTAKE_PID_INTEGRATOR_MAX = 2.0;

    /** Extra SmartDashboard keys for shooter/intake RPM and error (tuning). */
    public static final boolean IO_PID_TELEMETRY = true;

    // -----------------------------------------------------------------------
    // Intake / loader outputs (still open-loop on those motors)
    // -----------------------------------------------------------------------

    /** Intake motor voltage when feeding balls toward the shooter. */
    public static final double INTAKING_INTAKE_OUTPUT = -11.5;
    /**
     * Loader target voltage when feeding balls toward the shooter.
     * Sign controls direction; use negative to invert.
     */
    public static final double LOADER_MOTOR_TARGET_VOLTAGE = -6.0;
    /**
     * Loader duty cycle (0–1) corresponding to {@link #LOADER_MOTOR_TARGET_VOLTAGE}
     * on a 12 V bus.
     */
    public static final double INTAKING_LOADER_OUTPUT = LOADER_MOTOR_TARGET_VOLTAGE / 12.0;

    public static final double PREPARING_LOADER_OUTPUT = 0.0;

    /**
     * Loader duty cycle (0–1) for launching at fixed speed (opposite direction of
     * intake).
     */
    public static final double LAUNCHING_LOADER_OUTPUT = -INTAKING_LOADER_OUTPUT;

    /**
     * Wait 1 second for the shooter to spin super fast before throwing the ball
     * into it.
     */
    public static final double LAUNCH_SPIN_UP_SECONDS = 1.0;

    /**
     * How long to wait for the shooter to spin up when eating balls (0 means don't
     * wait).
     */
    public static final double INTAKE_SPIN_UP_SECONDS = 0;

    /**
     * How long to wait for the shooter to spin up during the robot's self-driving
     * mode.
     */
    public static final double INTAKE_AUTON_SPIN_UP_SECONDS = 2;

    /** Power to keep the shooter halfway spun up so it's ready quickly. */
    public static final double FLYWHEEL_SPIN_UP_50_VOLTAGE = 6.0;

    /**
     * How long the intake stays on when we do a pulsing wiggle (shoot sequence).
     * Total cycle is 2 seconds.
     */
    public static final double INTAKE_PULSE_ON_SECONDS = 1.0;
    /**
     * How long the intake stays off during the pulsing wiggle (shoot sequence).
     * Total cycle is 2 seconds.
     */
    public static final double INTAKE_PULSE_OFF_SECONDS = 1.0;

    /**
     * Logical grouping of CAN IDs for the IO / intake / loader motors. This
     * allows subsystems to accept a single argument instead of three separate
     * IDs, keeping wiring changes localized.
     */
    public static final class IoCanIdGroup {
      public final int ioMotorId;
      public final int intakeMotorId;
      public final int loaderMotorId;

      public IoCanIdGroup(int ioMotorId, int intakeMotorId, int loaderMotorId) {
        this.ioMotorId = ioMotorId;
        this.intakeMotorId = intakeMotorId;
        this.loaderMotorId = loaderMotorId;
      }
    }

    /** Default CAN ID group for the production robot. */
    public static final IoCanIdGroup IO_CAN_IDS = new IoCanIdGroup(IO_MOTOR_ID, INTAKE_MOTOR_ID, LOADER_MOTOR_ID);
  }

  /**
   * Timing and open-loop speeds for
   * {@link frc.robot.RobotContainer#autonomousCommand()}.
   * Drive segments use {@link frc.robot.commands.AutoDrive} with timeouts only
   * (not encoder distance). Tune times on the field.
   */
  public static final class AutoConstants {
    public static final double AUTO_INITIAL_SHOOT_SECONDS = 5.0;
    public static final double AUTO_FWD1_SPEED = 0.75;
    public static final double AUTO_FWD1_SECONDS = 0.5;
    public static final double AUTO_TURN1_ROTATION = -0.66;
    public static final double AUTO_TURN1_SECONDS = 0.5;
    public static final double AUTO_FWD2_SPEED = 0.75;
    public static final double AUTO_FWD2_SECONDS = 1.0;
    public static final double AUTO_TURN2_ROTATION = 0.64;
    public static final double AUTO_TURN2_SECONDS = 0.5;
    public static final double AUTO_FWD_INTAKE_SPEED = 0.75;
    public static final double AUTO_FWD_INTAKE_SECONDS = 0.87;
    public static final double AUTO_TURN3_ROTATION = 0.5;
    public static final double AUTO_TURN3_SECONDS = 0.5;
    public static final double AUTO_FINAL_SHOOT_SECONDS = 5.0;

    /**
     * Short presets for the SmartDashboard {@code Robot/Auto choices}
     * SendableChooser
     * (time-based only; tune on the field).
     */
    public static final double CHOOSER_SHOOT_ONLY_SECONDS = 4.0;
    public static final double CHOOSER_SIMPLE_FWD_SPEED = 0.55;
    public static final double CHOOSER_SIMPLE_FWD_SECONDS = 1.5;
    public static final double CHOOSER_SIMPLE_REV_SPEED = -0.45;
    public static final double CHOOSER_SIMPLE_REV_SECONDS = 1.5;
    public static final double CHOOSER_SHOOT_THEN_FWD_SHOOT_SECONDS = 3.0;
    public static final double CHOOSER_DRIVE_2S_SPEED = 0.5;
    public static final double CHOOSER_DRIVE_2S_SECONDS = 2.0;
  }

  public static final class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
    public static final double DRIVE_SCALING = 1.0;
    public static final double ROTATION_SCALING = 1.0;
    /**
     * Deadband for driver sticks (arcade drive). Values with absolute magnitude
     * below this are treated as zero to reduce drift.
     */
    public static final double DRIVE_DEADBAND = 0.08;
    /**
     * Threshold for treating a trigger as "pressed" for command bindings. This
     * keeps the behavior consistent across LT/RT usages.
     */
    public static final double TRIGGER_THRESHOLD = 0.5;
  }

  /**
   * Returns a formatted list of all CAN IDs for logging or display.
   */
  public static String getCanIdsList() {
    return String.join("\n",
        "========== CAN IDs ==========",
        "Drivetrain:",
        "  Left  leader:  " + DriveConstants.LEFT_LEADER_ID,
        "  Left  follower: " + DriveConstants.LEFT_FOLLOWER_ID,
        "  Right leader:  " + DriveConstants.RIGHT_LEADER_ID,
        "  Right follower: " + DriveConstants.RIGHT_FOLLOWER_ID,
        "",
        "IO / Loader:",
        "  IO motor:     " + IoConstants.IO_MOTOR_ID,
        "  Intake motor: " + IoConstants.INTAKE_MOTOR_ID,
        "  Loader motor: " + IoConstants.LOADER_MOTOR_ID,
        "=============================");
  }
}