// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import static frc.robot.Constants.OperatorConstants.*;

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

    autoChooser.addOption("(test) turn left 2s",
        new AutoDrive(driveSubsystem, 0, 0.1).withTimeout(2.0)
            .andThen(Commands.none()));

    autoChooser.addOption("(untested) Drive back & shoot preload left",
        new AutoDrive(driveSubsystem, -0.5, 0.0).withTimeout(2.0)
            .andThen(new AutoDrive(driveSubsystem, 0, 0.1).withTimeout(2.0))
            .andThen(ioSubsystem.commandLaunch().withTimeout(1.0))
            .andThen(Commands.none()));

    autoChooser.addOption("(untested) Drive back & shoot preload right",
        new AutoDrive(driveSubsystem, -0.5, 0.0).withTimeout(2.0)
            .andThen(new AutoDrive(driveSubsystem, 0, -0.1).withTimeout(2.0))
            .andThen(ioSubsystem.commandLaunch().withTimeout(1.0))
            .andThen(Commands.none()));
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

  private void configureBindings() {
    driveSubsystem.setDefaultCommand(new Drive(driveSubsystem, driverController));
    ioSubsystem.setDefaultCommand(ioSubsystem.commandIdle());

    // Left trigger SHOOTS: spin up shooter with encoder control and feed balls
    // (loader gated on shooter speed) while allowing normal driving.
    driverController.leftTrigger(TRIGGER_THRESHOLD).whileTrue(
        ioSubsystem.commandLaunch()
            .alongWith(ioSubsystem.commandIntakePulse())
            .alongWith(driveSubsystem.commandIntakeWiggle(0.4, 0.25)));

    // Right trigger INTAKES: run intake + loader only (no shooter spin).
    driverController.rightTrigger(TRIGGER_THRESHOLD).whileTrue(ioSubsystem.commandIntake());
    driverController.leftBumper().whileTrue(ioSubsystem.commandEject());
    driverController.x().onTrue(Commands.runOnce(ioSubsystem::toggleSpinUp50Requested));
    driverController.y().whileTrue(ioSubsystem.commandReverseFlywheelAndLoader());
    // B button: high-speed shoot with automatic forward/backward wiggle.
    driverController.b().whileTrue(
        ioSubsystem.commandHighSpeedLaunch()
            .alongWith(ioSubsystem.commandIntakePulse())
            .alongWith(driveSubsystem.commandIntakeWiggle(0.4, 0.25)));
    // Right bumper: ultra-speed shoot with same intake pulsing + wiggle.
    driverController.rightBumper().whileTrue(
        ioSubsystem.commandUltraSpeedLaunch()
            .alongWith(ioSubsystem.commandIntakePulse())
            .alongWith(driveSubsystem.commandIntakeWiggle(0.4, 0.25)));
    driverController.a().whileTrue(ioSubsystem.commandMaxSpin());
  }

  /**
   * Autonomous: start at center, drive toward our HUB to shooting range, then
   * launch into HUB.
   * Assumes robot is facing the alliance HUB. Tune CENTER_TO_SHOOT_DRIVE_METERS
   * for your shooter.
   */
  public Command autonomousCommand() {

    Command run = new AutoDrive(driveSubsystem, 0.5, 0.0).withTimeout(1.0);
    Command spin = ioSubsystem.commandIntakeAuton().withTimeout(7.0);
    return Commands.sequence(spin, run);

  }

  /** Returns the autonomous command selected on the dashboard. */
  public Command getAutonomousCommand() {
    return autonomousCommand();
  }
}
