// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;

/**
 * Drives the robot at a fixed speed and rotation for use in autonomous
 * routines.
 * Uses {@link edu.wpi.first.wpilibj.drive.DifferentialDrive#arcadeDrive}:
 * xSpeed
 * forward positive, zRotation counterclockwise positive [-1, 1]. Called every
 * cycle to satisfy MotorSafety. Pair with {@code .withTimeout(seconds)} to
 * bound duration.
 *
 * <p>
 * Example: drive straight for 2 seconds:
 * 
 * <pre>
 * new AutoDrive(driveSubsystem, 0.5, 0.0).withTimeout(2)
 * </pre>
 */
public class AutoDrive extends Command {

  private final CANDriveSubsystem driveSubsystem;
  private final double xSpeed;
  private final double zRotation;

  /**
   * @param driveSystem The drive subsystem
   * @param xSpeed      Forward speed [-1, 1]; positive = forward
   * @param zRotation   Rotation rate [-1, 1]; positive = rotate right
   */
  public AutoDrive(CANDriveSubsystem driveSystem, double xSpeed, double zRotation) {
    addRequirements(driveSystem);
    this.driveSubsystem = driveSystem;
    this.xSpeed = xSpeed;
    this.zRotation = zRotation;
  }

  @Override
  public void initialize() {
  }

  // Called every loop while scheduled; feeding driveArcade continuously
  // keeps the motor-safety watchdog satisfied
  @Override
  public void execute() {
    driveSubsystem.driveArcade(xSpeed, zRotation);
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stop();
  }

  // Never finishes on its own; use .withTimeout() to bound its duration
  @Override
  public boolean isFinished() {
    return false;
  }
}