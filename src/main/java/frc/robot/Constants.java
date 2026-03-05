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
  }

  public static final class IoConstants {
    public static final int IO_MOTOR_ID = 9;
    /**
     * Intake (12) – pulls fuel from floor/storage. Anti-clockwise = intake;
     * clockwise = spit out.
     */
    public static final int INTAKE_MOTOR_ID = 12;
    public static final int LOADER_MOTOR_ID = 19;

    public static final int IO_MOTOR_CURRENT_LIMIT = 60;
    public static final int INTAKE_MOTOR_CURRENT_LIMIT = 60;
    public static final int LOADER_MOTOR_CURRENT_LIMIT = 60;

    /** IO flywheel voltage when intaking from the floor/storage. */
    public static final double INTAKING_IO_VOLTAGE = 12;
    /** Intake motor voltage when intaking. */
    public static final double INTAKING_INTAKE_OUTPUT = -10;
    /** Loader duty cycle (0–1) when intaking. */
    public static final double INTAKING_LOADER_OUTPUT = 6.0 / 12.0;

    public static final double PREPARING_IO_VOLTAGE = -6;
    public static final double PREPARING_LOADER_OUTPUT = 0.0;

    /**
     * Flywheel (IO) voltage during launch; spin-up phase uses this before feeding
     * loader.
     */
    public static final double LAUNCHING_IO_VOLTAGE = 0;
    /**
     * Loader duty cycle (0–1) for launching at fixed speed (opposite direction of
     * intake).
     */
    public static final double LAUNCHING_LOADER_OUTPUT = -INTAKING_LOADER_OUTPUT;

    /** Delay (seconds) to spin up flywheel before feeding loader when launching. */
    public static final double LAUNCH_SPIN_UP_SECONDS = 0.5;

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
  }

  public static final class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
    public static final double DRIVE_SCALING = 0.7;
    public static final double ROTATION_SCALING = 0.8;
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