// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Motor controller IDs for drivetrain motors
    public static final int LEFT_LEADER_ID = 11;
    public static final int LEFT_FOLLOWER_ID = 8;
    public static final int RIGHT_LEADER_ID = 10;
    public static final int RIGHT_FOLLOWER_ID = 7;

    // Current limit for drivetrain motors. 60A is a reasonable maximum to reduce
    // likelihood of tripping breakers or damaging CIM motors
    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;

    // Physical geometry — measure for your robot
    /** Wheel diameter in meters (e.g. 6 inches ≈ 0.1524 m). */
    public static final double WHEEL_DIAMETER_METERS = 0.1524;
    /** Gear ratio from motor to wheel (e.g. 10.71 for standard KitBot gearbox). */
    public static final double GEAR_RATIO = 10.71;
  }

  public static final class IoConstants {
    public static final int IO_MOTOR_ID = 9;
    // public static final int SOMETHING_MOTOR_ID = 12;
    public static final int LOADER_MOTOR_ID = 19;

    public static final int IO_MOTOR_CURRENT_LIMIT = 60;
    public static final int LOADER_MOTOR_CURRENT_LIMIT = 60;

    // IO motor: voltage (V). Loader: duty cycle 0–1 (use setVoltage/set
    // respectively).
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
    // Port constants for driver and operator controllers. These should match the
    // values in the Joystick tab of the Driver Station software
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    // Scaling multipliers applied to joystick inputs to keep driving manageable
    public static final double DRIVE_SCALING = 0.7;
    public static final double ROTATION_SCALING = 0.8;
  }
}