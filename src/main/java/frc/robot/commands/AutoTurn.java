// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;
import static frc.robot.Constants.DriveConstants.DRIVE_TRACK_WIDTH_METERS;
import static frc.robot.Constants.DriveConstants.AUTO_TURN_DISTANCE_SCALAR;

/**
 * Turns the robot in place using drivetrain encoders.
 *
 * <p>This is an encoder-only turn (no gyro). It assumes an in-place turn where
 * the left and right sides travel equal and opposite distances.
 */
public class AutoTurn extends Command {
  private final CANDriveSubsystem driveSubsystem;
  private final double targetAngleDegrees;
  private final double turnSpeed;

  /**
   * @param driveSubsystem The drive subsystem (encoders will be used).
   * @param targetAngleDegrees Positive = turn right, negative = turn left.
   * @param turnSpeed Tank turn speed magnitude in [0, 1].
   */
  public AutoTurn(CANDriveSubsystem driveSubsystem, double targetAngleDegrees, double turnSpeed) {
    addRequirements(driveSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.targetAngleDegrees = targetAngleDegrees;
    this.turnSpeed = Math.abs(turnSpeed);
  }

  @Override
  public void initialize() {
    driveSubsystem.resetEncoders();
  }

  @Override
  public void execute() {
    double sign = Math.signum(targetAngleDegrees);
    double left = turnSpeed * sign;
    double right = -turnSpeed * sign;
    driveSubsystem.driveTank(left, right);
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    double rotationRadians = Math.toRadians(Math.abs(targetAngleDegrees));
    double requiredSideDistanceMeters =
        ((rotationRadians * DRIVE_TRACK_WIDTH_METERS) / 2.0) * AUTO_TURN_DISTANCE_SCALAR;

    double left = Math.abs(driveSubsystem.getLeftDistanceMeters());
    double right = Math.abs(driveSubsystem.getRightDistanceMeters());
    double avgSideDistance = (left + right) / 2.0;

    return avgSideDistance >= requiredSideDistanceMeters;
  }
}

