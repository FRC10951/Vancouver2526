// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.logging.RobotTelemetryLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * KitBot 2026 main robot class. Mode logic is handled by the command scheduler;
 * initialization and CAN ID reporting run in robotInit().
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    DataLogManager.start();
    RobotTelemetryLog.start();

    m_robotContainer = new RobotContainer();

    String canIds = Constants.getCanIdsList();
    System.out.println(canIds);
    SmartDashboard.putString("CAN IDs", canIds);

    HAL.report(tResourceType.kResourceType_Framework, 10);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    RobotTelemetryLog.recordBattery(RobotController.getBatteryVoltage());
  }

  @Override
  public void autonomousInit() {
    // Command is chosen in RobotContainer.getAutonomousCommand(): FMS alliance + station
    // when both are present, otherwise the SmartDashboard "Auto choices" selection.
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
      m_autonomousCommand = null;
    }
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
    if (m_robotContainer != null) {
      m_robotContainer.getIoSubsystem().stop();
      m_robotContainer.getDriveSubsystem().stop();
    }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void simulationPeriodic() {
    if (m_robotContainer != null) {
      m_robotContainer.getIoSubsystem().simulationPeriodic();
      m_robotContainer.getDriveSubsystem().simulationPeriodic();
    }
  }
}