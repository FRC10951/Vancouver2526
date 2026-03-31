package frc.robot;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    DriverStation.startDataLog(DataLogManager.getLog());

    m_robotContainer = new RobotContainer();

    String canIds = Constants.getCanIdsList();
    System.out.println(canIds);
    SmartDashboard.putString("Robot/Info/CAN IDs", canIds);
    SmartDashboard.putString("CAN IDs", canIds);

    HAL.report(tResourceType.kResourceType_Framework, 10);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    DashboardTelemetry.publishRobotState();
    if (m_robotContainer != null) {
      m_robotContainer.publishDashboardPeriodic();
    }
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    // go to pos
    // shoot!! !!!! !! !! !
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void simulationPeriodic() {
    DashboardTelemetry.publishSimulationBanner();
    SmartDashboard.putNumber("Robot/Sim time (s)", Timer.getFPGATimestamp());
  }
}