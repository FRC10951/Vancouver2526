// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.ElectricalConstants.BATTERY_CAUTION_VOLTS;
import static frc.robot.Constants.ElectricalConstants.MIN_BATTERY_VOLTS_FOR_CRITICAL_SHOT;

import edu.wpi.first.wpilibj.RobotController;

/**
 * Match-time electrical health used for operator feedback. Does not change motor
 * outputs; see {@link Constants.ElectricalConstants} for assumptions.
 */
public final class MatchReadiness {
  private MatchReadiness() {}

  public static double getBatteryVoltage() {
    return RobotController.getBatteryVoltage();
  }

  /** True when bus voltage is high enough to trust shooter regulation. */
  public static boolean isBatteryHealthyForShooter() {
    return getBatteryVoltage() >= MIN_BATTERY_VOLTS_FOR_CRITICAL_SHOT;
  }

  /**
   * True when voltage is in the caution band (below nominal but above critical).
   * Overload accepts a voltage for unit tests; the no-arg form uses the live
   * robot bus voltage.
   */
  public static boolean isBatteryInCautionBand(double batteryVolts) {
    return batteryVolts < BATTERY_CAUTION_VOLTS
        && batteryVolts >= MIN_BATTERY_VOLTS_FOR_CRITICAL_SHOT;
  }

  public static boolean isBatteryInCautionBand() {
    return isBatteryInCautionBand(getBatteryVoltage());
  }
}
