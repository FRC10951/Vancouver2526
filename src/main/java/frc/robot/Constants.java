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

    /**
     * Distance between wheel centers in meters (~22 in for KitBot AM14U).
     * CALIBRATE THIS: mark a spot, command a 360° turn, and adjust until
     * the robot returns to its starting heading.
     */
    public static final double TRACK_WIDTH_METERS = 0.559;

    /** Duty-cycle speed for straight-line driving in auto [0, 1]. */
    public static final double AUTO_DRIVE_SPEED = 0.4;
    /** Duty-cycle speed for point-turns in auto [0, 1]. Increase for faster turns; reduce if overshooting. */
    public static final double AUTO_TURN_SPEED = 0.6;
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

    /** IO flywheel voltage when intaking from the floor/storage (off). */
    public static final double INTAKING_IO_VOLTAGE = 0.0;
    /** Intake motor voltage when intaking (full power). */
    public static final double INTAKING_INTAKE_OUTPUT = 12.0;
    /** Loader duty cycle (0–1) when intaking (full power, opposite of launch direction). */
    public static final double INTAKING_LOADER_OUTPUT = -1.0;

    public static final double PREPARING_IO_VOLTAGE = -6;
    public static final double PREPARING_LOADER_OUTPUT = 0.0;

    /** IO flywheel voltage when launching toward the target (full power). */
    public static final double LAUNCHING_IO_VOLTAGE = 12.0;
    /** Loader duty cycle (0–1) for launching at fixed speed (full power). */
    public static final double LAUNCHING_LOADER_OUTPUT = 1.0;

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
    public static final IoCanIdGroup IO_CAN_IDS =
        new IoCanIdGroup(IO_MOTOR_ID, INTAKE_MOTOR_ID, LOADER_MOTOR_ID);
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