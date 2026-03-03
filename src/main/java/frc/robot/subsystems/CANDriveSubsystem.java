// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DriveConstants.*;

/**
 * Tank (differential) drive subsystem using CAN SparkMax controllers.
 *
 * <p>Supports both tank drive (independent left/right sticks) and arcade drive
 * (forward + rotation), making it easy to switch control styles in {@code Drive.java}.
 */
public class CANDriveSubsystem extends SubsystemBase {

  // --- Motors ---
  private final SparkMax leftLeader;
  private final SparkMax leftFollower;
  private final SparkMax rightLeader;
  private final SparkMax rightFollower;

  private final DifferentialDrive drive;

  // --- Encoders ---
  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;

  private static final double METERS_PER_ROTATION =
      (Math.PI * WHEEL_DIAMETER_METERS) / GEAR_RATIO;

  public CANDriveSubsystem() {
    // Create brushed motors for a KitBot-style CIM drivetrain
    leftLeader   = new SparkMax(LEFT_LEADER_ID,   MotorType.kBrushed);
    leftFollower  = new SparkMax(LEFT_FOLLOWER_ID,  MotorType.kBrushed);
    rightLeader  = new SparkMax(RIGHT_LEADER_ID,  MotorType.kBrushed);
    rightFollower = new SparkMax(RIGHT_FOLLOWER_ID, MotorType.kBrushed);

    drive = new DifferentialDrive(leftLeader, rightLeader);

    // Longer CAN timeout is fine here because configuration only runs once at init
    leftLeader.setCANTimeout(250);
    rightLeader.setCANTimeout(250);
    leftFollower.setCANTimeout(250);
    rightFollower.setCANTimeout(250);

    // Left leader: invert so that positive values drive both sides forward
    SparkMaxConfig leftLeaderConfig = new SparkMaxConfig();
    leftLeaderConfig.voltageCompensation(12);
    leftLeaderConfig.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);
    leftLeaderConfig.inverted(true);
    leftLeader.configure(leftLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Right leader: not inverted
    SparkMaxConfig rightLeaderConfig = new SparkMaxConfig();
    rightLeaderConfig.voltageCompensation(12);
    rightLeaderConfig.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);
    rightLeader.configure(rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Followers mirror their respective leaders
    SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
    leftFollowerConfig.follow(leftLeader);
    leftFollower.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();
    rightFollowerConfig.follow(rightLeader);
    rightFollower.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Built-in encoders
    leftEncoder  = leftLeader.getEncoder();
    rightEncoder = rightLeader.getEncoder();
  }

  // ---------------------------------------------------------------------------
  // Periodic
  // ---------------------------------------------------------------------------

  @Override
  public void periodic() {
    // Publish useful diagnostics to SmartDashboard
    SmartDashboard.putNumber("Left Distance (m)",    getLeftDistanceMeters());
    SmartDashboard.putNumber("Right Distance (m)",   getRightDistanceMeters());
    SmartDashboard.putNumber("Left Velocity (m/s)",  getLeftVelocityMetersPerSecond());
    SmartDashboard.putNumber("Right Velocity (m/s)", getRightVelocityMetersPerSecond());
  }

  // ---------------------------------------------------------------------------
  // Drive methods
  // ---------------------------------------------------------------------------

  /**
   * Tank drive: each side is controlled independently.
   * Positive values drive each side forward.
   *
   * @param leftSpeed  Speed for the left side  [-1, 1]
   * @param rightSpeed Speed for the right side [-1, 1]
   */
  public void driveTank(double leftSpeed, double rightSpeed) {
    drive.tankDrive(leftSpeed, rightSpeed);
  }

  /**
   * Arcade drive: one stick for forward/backward, one for rotation.
   *
   * @param xSpeed    Forward/backward speed [-1, 1]
   * @param zRotation Rotation rate          [-1, 1]
   */
  public void driveArcade(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation);
  }

  /**
   * Sets raw voltage on each side of the drivetrain.
   * Useful for autonomous routines that need precise distance control.
   *
   * @param leftVolts  Voltage for the left side  [-12, 12]
   * @param rightVolts Voltage for the right side [-12, 12]
   */
  public void driveVolts(double leftVolts, double rightVolts) {
    leftLeader.setVoltage(leftVolts);
    rightLeader.setVoltage(rightVolts);
    drive.feed(); // Prevent motor safety watchdog from cutting power
  }

  /** Stops all drivetrain motors immediately. */
  public void stop() {
    drive.stopMotor();
  }

  // ---------------------------------------------------------------------------
  // Encoder helpers
  // ---------------------------------------------------------------------------

  /**
   * Returns the distance traveled by the left side in meters.
   */
  public double getLeftDistanceMeters() {
    return leftEncoder.getPosition() * METERS_PER_ROTATION;
  }

  /**
   * Returns the distance traveled by the right side in meters.
   */
  public double getRightDistanceMeters() {
    return rightEncoder.getPosition() * METERS_PER_ROTATION;
  }

  /**
   * Returns the average distance traveled by both sides in meters.
   * Convenient for straight-line distance calculations in auto.
   */
  public double getAverageDistanceMeters() {
    return (getLeftDistanceMeters() + getRightDistanceMeters()) / 2.0;
  }

  /**
   * Returns left-side velocity in meters per second.
   */
  public double getLeftVelocityMetersPerSecond() {
    return (leftEncoder.getVelocity() / 60.0) * METERS_PER_ROTATION;
  }

  /**
   * Returns right-side velocity in meters per second.
   */
  public double getRightVelocityMetersPerSecond() {
    return (rightEncoder.getVelocity() / 60.0) * METERS_PER_ROTATION;
  }

  /** Resets both drive encoders to zero. */
  public void resetEncoders() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }
}