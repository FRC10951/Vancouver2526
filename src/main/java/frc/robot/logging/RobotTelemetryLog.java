// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.logging;

import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

/**
 * Double-precision fields for AdvantageScope / WPILib log replay ({@link DataLogManager}). Call
 * {@link #start()} once after {@link DataLogManager#start()}.
 */
public final class RobotTelemetryLog {
  private static DoubleLogEntry shooterRpm;
  private static DoubleLogEntry shooterTargetRpm;
  private static DoubleLogEntry intakeRpm;
  private static DoubleLogEntry intakeTargetRpm;
  private static DoubleLogEntry batteryVolts;
  private static boolean initialized;

  private RobotTelemetryLog() {}

  /** Registers log topics on the active {@link DataLog}. Safe to call once from {@code robotInit}. */
  public static void start() {
    if (initialized) {
      return;
    }
    DataLog log = DataLogManager.getLog();
    shooterRpm = new DoubleLogEntry(log, "/RealOutputs/IO/ShooterRpm");
    shooterTargetRpm = new DoubleLogEntry(log, "/RealOutputs/IO/ShooterTargetRpm");
    intakeRpm = new DoubleLogEntry(log, "/RealOutputs/IO/IntakeRpm");
    intakeTargetRpm = new DoubleLogEntry(log, "/RealOutputs/IO/IntakeTargetRpm");
    batteryVolts = new DoubleLogEntry(log, "/RealOutputs/Electrical/BatteryVolts");
    initialized = true;
  }

  public static boolean isInitialized() {
    return initialized;
  }

  public static void recordIo(
      double shooterRpmValue,
      double shooterTargetValue,
      double intakeRpmValue,
      double intakeTargetValue) {
    if (!initialized) {
      return;
    }
    long t = WPIUtilJNI.now();
    shooterRpm.append(shooterRpmValue, t);
    shooterTargetRpm.append(shooterTargetValue, t);
    intakeRpm.append(intakeRpmValue, t);
    intakeTargetRpm.append(intakeTargetValue, t);
  }

  public static void recordBattery(double volts) {
    if (!initialized) {
      return;
    }
    batteryVolts.append(volts, WPIUtilJNI.now());
  }
}
