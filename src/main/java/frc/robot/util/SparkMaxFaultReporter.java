// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Publishes SPARK MAX active/sticky faults to SmartDashboard and emits a
 * DriverStation warning when the sticky fault bitmask changes (e.g. new CAN or
 * temperature fault). Useful during matches for spotting browning-related or
 * wiring issues.
 */
public final class SparkMaxFaultReporter {
  private final String dashboardKeyPrefix;
  private int lastStickyRawBits = -1;

  /**
   * @param dashboardKeyPrefix e.g. {@code "IO/Flywheel"} maps to keys like {@code
   *     IO/Flywheel/sticky fault bits}
   */
  public SparkMaxFaultReporter(String dashboardKeyPrefix) {
    this.dashboardKeyPrefix = dashboardKeyPrefix;
  }

  /** Call from subsystem {@code periodic()}. */
  public void reportPeriodic(SparkMax motor) {
    SparkBase.Faults active = motor.getFaults();
    SparkBase.Faults sticky = motor.getStickyFaults();

    SmartDashboard.putNumber(dashboardKeyPrefix + "/fault raw", active.rawBits);
    SmartDashboard.putString(dashboardKeyPrefix + "/faults", formatFaults(active));
    SmartDashboard.putNumber(dashboardKeyPrefix + "/sticky fault raw", sticky.rawBits);
    SmartDashboard.putString(dashboardKeyPrefix + "/sticky faults", formatFaults(sticky));
    SmartDashboard.putBoolean(dashboardKeyPrefix + "/has active fault", motor.hasActiveFault());
    SmartDashboard.putBoolean(dashboardKeyPrefix + "/has sticky fault", motor.hasStickyFault());

    if (sticky.rawBits != lastStickyRawBits) {
      if (sticky.rawBits != 0) {
        DriverStation.reportWarning(
            "["
                + dashboardKeyPrefix
                + "] Sticky SPARK fault(s): "
                + formatFaults(sticky)
                + " (raw 0x"
                + Integer.toHexString(sticky.rawBits)
                + ")",
            false);
      }
      lastStickyRawBits = sticky.rawBits;
    }
  }

  /** Visible for unit tests (same package) and diagnostics. */
  static String formatFaults(SparkBase.Faults f) {
    if (f == null || f.rawBits == 0) {
      return "none";
    }
    StringBuilder sb = new StringBuilder();
    if (f.can) {
      sb.append("CAN ");
    }
    if (f.temperature) {
      sb.append("temp ");
    }
    if (f.gateDriver) {
      sb.append("gateDriver ");
    }
    if (f.firmware) {
      sb.append("firmware ");
    }
    if (f.sensor) {
      sb.append("sensor ");
    }
    if (f.motorType) {
      sb.append("motorType ");
    }
    if (f.escEeprom) {
      sb.append("escEeprom ");
    }
    if (f.other) {
      sb.append("other ");
    }
    return sb.toString().trim();
  }
}
