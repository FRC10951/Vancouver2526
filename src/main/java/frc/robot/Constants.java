// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * Robot-wide constants: CAN IDs, current limits, voltages, and operator scaling.
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
    /** Intake (12) – pulls fuel from floor/storage. Anti-clockwise = intake; clockwise = spit out. */
    public static final int INTAKE_MOTOR_ID = 12;
    public static final int LOADER_MOTOR_ID = 19;

    public static final int IO_MOTOR_CURRENT_LIMIT = 60;
    public static final int LOADER_MOTOR_CURRENT_LIMIT = 60;

    public static final double INTAKING_IO_VOLTAGE = -12;
    /** Loader duty cycle (0–1) for intaking. */
    public static final double INTAKING_LOADER_OUTPUT = 10.0 / 12.0;

    public static final double PREPARING_IO_VOLTAGE = -6;
    public static final double PREPARING_LOADER_OUTPUT = 0.0;

    public static final double LAUNCHING_IO_VOLTAGE = 12;
    /** Loader duty cycle (0–1) for launching at fixed speed. */
    public static final double LAUNCHING_LOADER_OUTPUT = 10.0 / 12.0;
  }

  public static final class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
    public static final double DRIVE_SCALING = 0.7;
    public static final double ROTATION_SCALING = 0.8;
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
        "============================="
    );
  }
}