// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * Robot settings! This file stores all the important numbers for our robot,
 * like which motor is which and how fast things should spin.
 * Think of it as the robot's rulebook.
 */
public final class Constants {
  public static final class DriveConstants {
    // ID numbers so the robot knows which driving motor is which!
    public static final int LEFT_LEADER_ID = 11;
    public static final int LEFT_FOLLOWER_ID = 8;
    public static final int RIGHT_LEADER_ID = 10;
    public static final int RIGHT_FOLLOWER_ID = 7;

    // How much power the driving motors are allowed to use so they don't break.
    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;

    /** How wide our robot's wheels are, measured in meters. */
    public static final double WHEEL_DIAMETER_METERS = 0.1524;

    /**
     * How many times the motor spins inside to make the wheel spin once outside.
     */
    public static final double GEAR_RATIO = 10.71;

    /** How fast the intake wiggles back and forth to unjam balls. */
    public static final double INTAKE_WIGGLE_SPEED = 0.5;

    /** How long each wiggle lasts in seconds. */
    public static final double INTAKE_WIGGLE_HALF_PERIOD = 0.1;
  }

  public static final class IoConstants {
    // The motor that shoots the balls!
    public static final int FLYWHEEL_MOTOR_ID = 9;

    /**
     * The motor that sucks balls in from the floor.
     * Spinning one way eats the ball, spinning the other way spits it out.
     */
    public static final int INTAKE_MOTOR_ID = 12;

    // The motor that moves the balls from the intake to the shooter.
    public static final int LOADER_MOTOR_ID = 19;

    // Limits to protect the motors from getting too tired or hot.
    public static final int FLYWHEEL_MOTOR_CURRENT_LIMIT = 60;
    public static final int INTAKE_MOTOR_CURRENT_LIMIT = 60;
    public static final int LOADER_MOTOR_CURRENT_LIMIT = 60;

    // -----------------------------------------------------------------------
    // Shooter and Intake speeds (measured in RPM, or Spins-Per-Minute!)
    // -----------------------------------------------------------------------

    /** Normal speed for the shooter. */
    public static final double SHOOTER_TARGET_SPEED_INTAKE_RPM = 1500.0;
    /** Speed for a normal shot (like a free throw). */
    public static final double SHOOTER_TARGET_SPEED_LAUNCH_RPM = 1500.0;
    /** Speed for a fast, powerful shot. */
    public static final double SHOOTER_TARGET_SPEED_HIGH_RPM = 2500.0;
    /** Speed for a super long-distance shot! */
    public static final double SHOOTER_TARGET_SPEED_ULTRA_RPM = 3800.0;
    /** A slower speed just to keep the shooter warmed up and ready. */
    public static final double SHOOTER_TARGET_SPEED_SPINUP50_RPM = 1000.0;
    /** Speed for the shooter when we pull the right trigger. */
    public static final double SHOOTER_TARGET_SPEED_TOGGLE_RPM = SHOOTER_TARGET_SPEED_INTAKE_RPM;

    /** A little math trick to help the shooter reach the exact right speed. */
    public static final double SHOOTER_KP = 0.003;

    /** Use 100% full power until the shooter is almost at the target speed! */
    public static final double SHOOTER_SPINUP_THRESHOLD_FRACTION = 0.9;

    /** The absolute maximum power the shooter is allowed to use (12 volts). */
    public static final double SHOOTER_MAX_VOLTAGE = 12.0;

    /** The normal amount of power needed to keep the shooter spinning perfectly. */
    public static final double SHOOTER_HOLD_BASE_VOLTAGE = 7.0;

    // -----------------------------------------------------------------------
    // Intake and Loader power levels (using battery voltage)
    // -----------------------------------------------------------------------

    /** Power level for sucking balls in from the floor. */
    public static final double INTAKING_INTAKE_OUTPUT = -6.0;

    /** Power level for moving balls inside the robot. */
    public static final double LOADER_MOTOR_TARGET_VOLTAGE = -6.0;

    /**
     * Math trick to turn the 12-volt battery power into a simple 0-to-1 percentage.
     */
    public static final double INTAKING_LOADER_OUTPUT = LOADER_MOTOR_TARGET_VOLTAGE / 12.0;

    /** Power level when we are just waiting to shoot (0 means stop). */
    public static final double PREPARING_LOADER_OUTPUT = 0.0;

    /** Pushes the ball into the shooter really fast! */
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

    /** How long the intake stays on when we do a pulsing wiggle (shoot sequence). Total cycle is 2 seconds. */
    public static final double INTAKE_PULSE_ON_SECONDS = 1.0;
    /** How long the intake stays off during the pulsing wiggle (shoot sequence). Total cycle is 2 seconds. */
    public static final double INTAKE_PULSE_OFF_SECONDS = 1.0;

    /**
     * A little helper bundle! It groups the IDs for the shooter, intake, and loader
     * together so we don't mix them up.
     */
    public static final class IoCanIdGroup {
      public final int flywheelMotorId;
      public final int intakeMotorId;
      public final int loaderMotorId;

      public IoCanIdGroup(int flywheelMotorId, int intakeMotorId, int loaderMotorId) {
        this.flywheelMotorId = flywheelMotorId;
        this.intakeMotorId = intakeMotorId;
        this.loaderMotorId = loaderMotorId;
      }
    }

    /** The official ID numbers for our robot! */
    public static final IoCanIdGroup IO_CAN_IDS = new IoCanIdGroup(FLYWHEEL_MOTOR_ID, INTAKE_MOTOR_ID, LOADER_MOTOR_ID);
  }

  /**
   * Settings for when the robot drives itself (Autonomous mode)!
   * The robot has to know how far to drive and how fast to go without a human
   * driver.
   */
  public static final class AutoConstants {
    /** How far to drive forward before we stop and shoot the ball. */
    public static final double CENTER_TO_SHOOT_DRIVE_METERS = 2.25;

    /** How fast to drive while doing the self-driving routine. */
    public static final double CENTER_TO_SHOOT_SPEED = 0.6;

    /** How many seconds to run the shooter to throw the ball. */
    public static final double CENTER_TO_SHOOT_LAUNCH_SECONDS = 3.0;

    /** How long to shoot the ball in the simple self-driving routine. */
    public static final double BASIC_SHOOT_SECONDS = 3.0;

    /** How fast the robot spins to the right during the simple routine. */
    public static final double TURN_30_STARBOARD_SPEED = 0.35;

    /** How many seconds it takes to spin to the right. */
    public static final double TURN_30_STARBOARD_SECONDS = 1.2;

    /** How long to drive forward and suck up balls at the same time. */
    public static final double DRIVE_AND_INTAKE_SECONDS = 3.0;

    /** How fast to drive while sucking up balls. */
    public static final double DRIVE_AND_INTAKE_SPEED = 0.5;
  }

  /** Settings for the Xbox controllers we use to drive. */
  public static final class OperatorConstants {
    // Which USB port the driver's controller is plugged into.
    public static final int DRIVER_CONTROLLER_PORT = 0;
    // Which USB port the helper's controller is plugged into.
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    // Multipliers to make the robot drive faster or slower.
    public static final double DRIVE_SCALING = 1.0;
    public static final double ROTATION_SCALING = 1.0;

    /**
     * Ignores tiny accidental bumps on the joysticks so the robot doesn't drift
     * when we let go of the controller.
     */
    public static final double DRIVE_DEADBAND = 0.08;

    /**
     * How hard you have to pull the controller trigger to make it actually work.
     */
    public static final double TRIGGER_THRESHOLD = 0.5;
  }

  /**
   * Prints out a nice, neat list of all our motor ID numbers so we can
   * double-check them!
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
        "  Flywheel motor: " + IoConstants.FLYWHEEL_MOTOR_ID,
        "  Intake motor: " + IoConstants.INTAKE_MOTOR_ID,
        "  Loader motor: " + IoConstants.LOADER_MOTOR_ID,
        "=============================");
  }
}