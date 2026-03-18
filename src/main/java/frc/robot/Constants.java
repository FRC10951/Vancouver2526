// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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

    /** Speed of the back-and-forth wiggle motion [0, 1]. */
    public static final double INTAKE_WIGGLE_SPEED = 0.6;
    /** Half-period duration (seconds) of the wiggle motion. */
    public static final double INTAKE_WIGGLE_HALF_PERIOD = 0.1;
  }

  public static final class IoConstants {
    public static final int IO_MOTOR_ID = 9;
    /**
     * Intake (12) – pulls fuel from floor/storage. Anti-clockwise = intake;
     * clockwise = spit out.
     */
    public static final int INTAKE_MOTOR_ID = 12;
    public static final int LOADER_MOTOR_ID = 19;
    // intake variable swaped with lanch variable
    public static final int IO_MOTOR_CURRENT_LIMIT = 60;
    public static final int INTAKE_MOTOR_CURRENT_LIMIT = 60;
    public static final int LOADER_MOTOR_CURRENT_LIMIT = 60;

    // -----------------------------------------------------------------------
    // Shooter / intake speed control (encoder-based)
    // -----------------------------------------------------------------------

    /** Base shooter speed (RPM) used for intake/prepare helpers. */
    public static final double SHOOTER_TARGET_SPEED_INTAKE_RPM = 2000.0;
    /** Shooter speed (RPM) for normal launch (left trigger). */
    public static final double SHOOTER_TARGET_SPEED_LAUNCH_RPM = 1500.0; // hub
    /** Shooter speed (RPM) for high-speed launch (B button). */
    public static final double SHOOTER_TARGET_SPEED_HIGH_RPM = 2500.0; // 2m
    /** Shooter speed (RPM) for ultra-speed launch (right bumper). */
    public static final double SHOOTER_TARGET_SPEED_ULTRA_RPM = 3800.0; // trench
    /** Target shooter speed (RPM) for the 50% spin-up toggle. */
    public static final double SHOOTER_TARGET_SPEED_SPINUP50_RPM = 1000.0;
    /** Target shooter speed (RPM) when using the right-trigger toggle. */
    public static final double SHOOTER_TARGET_SPEED_TOGGLE_RPM = SHOOTER_TARGET_SPEED_INTAKE_RPM;

    /** Proportional gain for shooter speed control (simple P loop). */
    public static final double SHOOTER_KP = 0.003;
    /**
     * Fraction of target speed below which we apply max voltage to spin up quickly.
     * For example, 0.8 means full voltage until 80% of target speed is reached.
     */
    public static final double SHOOTER_SPINUP_THRESHOLD_FRACTION = 0.9;

    /** Maximum voltage the shooter is ever commanded to (absolute value). */
    public static final double SHOOTER_MAX_VOLTAGE = 12.0;

    /**
     * Nominal voltage used as a baseline during speed holding; P-control adjusts
     * around this.
     */
    public static final double SHOOTER_HOLD_BASE_VOLTAGE = 7.0;

    // -----------------------------------------------------------------------
    // Intake / loader outputs (still open-loop on those motors)
    // -----------------------------------------------------------------------

    /** Intake motor voltage when feeding balls toward the shooter. */
    public static final double INTAKING_INTAKE_OUTPUT = -6.0;
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

    /** Delay (seconds) to spin up shooter before feeding loader when launching. */
    public static final double LAUNCH_SPIN_UP_SECONDS = 1.0;
    /**
     * Delay (seconds) to spin up IO flywheel before intake/loader when intaking.
     */
    public static final double INTAKE_SPIN_UP_SECONDS = 0;
    public static final double INTAKE_AUTON_SPIN_UP_SECONDS = 2; // Intake spinup for auton specifically
    /**
     * IO motor voltage for "spin up 50%" toggle (X button). 50% of typical 12 V.
     */
    public static final double IO_SPIN_UP_50_VOLTAGE = 6.0;

    /** Duration (seconds) the intake is ON during a pulse cycle. */
    public static final double INTAKE_PULSE_ON_SECONDS = 0.5;
    /** Duration (seconds) the intake is OFF during a pulse cycle. */
    public static final double INTAKE_PULSE_OFF_SECONDS = 0.2;

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
   * Autonomous constants based on FRC 2026 REBUILT field.
   * Field: 317.7 in × 651.2 in. Center line bisects the 651.2 in length.
   * HUB is 158.6 in (~4.03 m) from each alliance wall → center to HUB ≈ 167 in
   * (4.24 m).
   * We drive from center toward our HUB and stop at estimated shooting range (~2
   * m in front of HUB).
   */
  public static final class AutoConstants {
    /**
     * Drive distance from center line to shooting position (meters). Tune for your
     * shooter range.
     */
    public static final double CENTER_TO_SHOOT_DRIVE_METERS = 2.25;
    /** Forward speed for center-to-shoot drive [0, 1]. */
    public static final double CENTER_TO_SHOOT_SPEED = 0.6;
    /** How long to run the launcher to shoot preload (seconds). */
    public static final double CENTER_TO_SHOOT_LAUNCH_SECONDS = 3.0;

    /** Basic auto: shoot duration (seconds). */
    public static final double BASIC_SHOOT_SECONDS = 3.0;
    /**
     * Basic auto: turn 30° starboard (right) — rotation rate [0, 1]. Positive =
     * right.
     */
    public static final double TURN_30_STARBOARD_SPEED = 0.35;
    /** Basic auto: time (seconds) to turn ~30° starboard. Tune to match robot. */
    public static final double TURN_30_STARBOARD_SECONDS = 1.2;
    /** Basic auto: drive forward while intaking — duration (seconds). */
    public static final double DRIVE_AND_INTAKE_SECONDS = 3.0;
    /** Basic auto: forward speed [0, 1] during drive-and-intake. */
    public static final double DRIVE_AND_INTAKE_SPEED = 0.5;
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