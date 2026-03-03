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

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final CANDriveSubsystem driveSubsystem = new CANDriveSubsystem();
  private final IoSubsystem ioSubsystem = new IoSubsystem();

  // The driver's controller
  private final CommandXboxController driverController = new CommandXboxController(
      DRIVER_CONTROLLER_PORT);

  // The operator's controller
  private final CommandXboxController operatorController = new CommandXboxController(
      OPERATOR_CONTROLLER_PORT);

  // The autonomous chooser
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureBindings();

    autoChooser.setDefaultOption("Do Nothing", Commands.none());
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
    // Default commands: drive runs continuously; IO stays stopped until LT/RT are
    // held.
    driveSubsystem.setDefaultCommand(new Drive(driveSubsystem, driverController));
    ioSubsystem.setDefaultCommand(ioSubsystem.commandStop());

    // --- DRIVER: drive subsystem only (sticks + B). IO only via LT/RT. ---
    // LT = intake only (IO). RT = shoot only (IO). One trigger, one system.
    driverController.leftTrigger(0.5).whileTrue(ioSubsystem.commandIntake());
    driverController.rightTrigger(0.5).whileTrue(ioSubsystem.commandLaunch());

    // B = drive only (reset encoders)
    driverController.b()
        .onTrue(driveSubsystem.runOnce(() -> driveSubsystem.resetEncoders()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
