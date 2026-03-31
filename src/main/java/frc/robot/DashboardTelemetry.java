package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Centralized SmartDashboard / NetworkTables keys for pit and driver-station displays.
 * Uses {@code Robot/...}, {@code Drive/...}, and {@code IO/...} prefixes so widgets group cleanly
 * in Shuffleboard and the Simulation GUI.
 */
public final class DashboardTelemetry {

  private DashboardTelemetry() {}

  private static boolean lastBrownout = false;

  /** Call from {@link Robot#robotPeriodic()} after the command scheduler runs. */
  public static void publishRobotState() {
    SmartDashboard.putNumber("Robot/Battery (V)", RobotController.getBatteryVoltage());
    boolean brownout = RobotController.isBrownedOut();
    SmartDashboard.putBoolean("Robot/Simulation", RobotBase.isSimulation());
    SmartDashboard.putBoolean("Robot/Brownout", brownout);
    SmartDashboard.putBoolean("Robot/FMS attached", DriverStation.isFMSAttached());
    SmartDashboard.putBoolean("Robot/DS attached", DriverStation.isDSAttached());

    String mode = "Disabled";
    if (DriverStation.isEnabled()) {
      if (DriverStation.isAutonomous()) {
        mode = "Autonomous";
      } else if (DriverStation.isTeleop()) {
        mode = "Teleop";
      } else if (DriverStation.isTest()) {
        mode = "Test";
      }
    }
    SmartDashboard.putString("Robot/Mode", mode);

    double matchTime = DriverStation.getMatchTime();
    SmartDashboard.putNumber("Robot/Match time (s)", matchTime);

    SmartDashboard.putString(
        "Robot/Alliance",
        DriverStation.getAlliance().map(Alliance::name).orElse("Unknown"));
    SmartDashboard.putNumber(
        "Robot/Station",
        DriverStation.getLocation().isPresent() ? DriverStation.getLocation().getAsInt() : -1);

    // On first transition into brownout, re-log CAN IDs to help pit crews correlate wiring.
    if (brownout && !lastBrownout) {
      String canIds = Constants.getCanIdsList();
      DriverStation.reportWarning("Brownout detected; current CAN mapping:\n" + canIds, false);
    }
    lastBrownout = brownout;
  }

  /** Extra hints while running in desktop simulation. */
  public static void publishSimulationBanner() {
    if (!RobotBase.isSimulation()) {
      return;
    }
    SmartDashboard.putString(
        "Robot/Sim hint",
        "Desktop sim: use Sim GUI joysticks or USB gamepad; connect Shuffleboard to 127.0.0.1");
  }
}
