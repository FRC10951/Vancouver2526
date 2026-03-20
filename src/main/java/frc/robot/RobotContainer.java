// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import static frc.robot.Constants.OperatorConstants.*;
import static frc.robot.Constants.DriveConstants.*;

import frc.robot.commands.AutoDrive;
import frc.robot.commands.Drive;

import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.IoSubsystem;

//import frc.robot.subsystems.IoSubsystem.commandLaunch;

/**
 * Robot container: subsystems, driver/operator controllers, button bindings,
 * and autonomous chooser. See README.md for control layout and CAN IDs.
 */
public class RobotContainer {
  private final CANDriveSubsystem driveSubsystem = new CANDriveSubsystem();
  private final IoSubsystem ioSubsystem = new IoSubsystem();
  private final CommandXboxController driverController = new CommandXboxController(DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController = new CommandXboxController(OPERATOR_CONTROLLER_PORT);
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();

    autoChooser.setDefaultOption("Do Nothing", Commands.none());
    // autoChooser.addOption("Center → Drive & Shoot",
    // centerDriveAndShootCommand());
    autoChooser.addOption("Drive Forward 2s", new AutoDrive(driveSubsystem, 0.5, 0.0).withTimeout(2.0));
  }

  public CANDriveSubsystem getDriveSubsystem() {
    return driveSubsystem;
  }

  public IoSubsystem getIoSubsystem() {
    return ioSubsystem;
  }

  public CommandXboxController getOperatorController() {
    return operatorController;
  }

  /**
   * Helper to wrap any launch command with the required intake pulsing and
   * drivetrain wiggle.
   */
  private Command createShootingSequence(Command launchCommand) {
    return Commands.parallel(
        launchCommand,
        ioSubsystem.commandIntakePulse(),
        driveSubsystem.commandIntakeWiggle(INTAKE_WIGGLE_SPEED, INTAKE_WIGGLE_HALF_PERIOD));
  }

  private void configureBindings() {
    driveSubsystem.setDefaultCommand(new Drive(driveSubsystem, driverController));
    ioSubsystem.setDefaultCommand(ioSubsystem.commandIdle());

    // Left trigger SHOOTS: spin up shooter with encoder control and feed balls
    // (loader gated on shooter speed) while allowing normal driving.
    driverController.leftTrigger(TRIGGER_THRESHOLD).whileTrue(
        createShootingSequence(ioSubsystem.commandLaunch()));

    // Right trigger INTAKES: run intake + loader only (no shooter spin).
    driverController.rightTrigger(TRIGGER_THRESHOLD).whileTrue(ioSubsystem.commandIntake());
    driverController.leftBumper().whileTrue(ioSubsystem.commandEject());
    driverController.x().onTrue(Commands.runOnce(ioSubsystem::toggleSpinUp50Requested));
    driverController.y().whileTrue(ioSubsystem.commandReverseFlywheelAndLoader());
    // B button: high-speed shoot with automatic forward/backward wiggle.
    driverController.b().whileTrue(
        createShootingSequence(ioSubsystem.commandHighSpeedLaunch()));
    // Right bumper: ultra-speed shoot with same intake pulsing + wiggle.
    driverController.rightBumper().whileTrue(
        createShootingSequence(ioSubsystem.commandUltraSpeedLaunch()));
    driverController.a().whileTrue(ioSubsystem.commandMaxSpin());
  }

  /**
   * Autonomous: shoot first, then drive forward for one second (arcade drive, no
   * rotation).
   * Tank drivetrain driven via arcade: xSpeed = forward, zRotation = 0 for
   * straight.
   */
  public Command autonomousCommand() {
    return Commands.sequence(
        // Shoot initial 8 fuel;
        ioSubsystem.commandLaunch().withTimeout(5.0),
        // Drive forward 0.5 meters;
        new AutoDrive(driveSubsystem, 0.75, 0.0).withTimeout(0.5),
        // Rotate 90 degrees to the right;
        new AutoDrive(driveSubsystem, 0.0, -0.66).withTimeout(0.5),
        // Drive forward 0.5 meters;
        new AutoDrive(driveSubsystem, 0.75, 0.0).withTimeout(1.0),
        // Rotate 90 degrees to the right;
        new AutoDrive(driveSubsystem, 0.0, 0.73).withTimeout(0.5),
        // Drive forward 2.8 meters;
        new AutoDrive(driveSubsystem, 0.75, 0.0).withTimeout(1.1));
  }

  /** Returns the autonomous command selected on the dashboard. */
  public Command getAutonomousCommand() {
    return autonomousCommand();
  }
}
