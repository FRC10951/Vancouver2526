// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

/**
 * Pure helpers for IO speed control; easy to unit test without hardware.
 */
public final class IoControlMath {
  private IoControlMath() {}

  /** Clamps {@code value} to {@code [-maxAbs, maxAbs]}. */
  public static double clampSymmetric(double value, double maxAbs) {
    if (value > maxAbs) {
      return maxAbs;
    }
    if (value < -maxAbs) {
      return -maxAbs;
    }
    return value;
  }

  /**
   * Shooter gate: true when current RPM is at or above the fraction of target used
   * for feed/loader enable (positive target RPM).
   */
  public static boolean shooterAtOrAboveFractionOfTarget(
      double currentRpm, double targetRpm, double thresholdFraction) {
    if (targetRpm <= 0.0) {
      return false;
    }
    return currentRpm >= targetRpm * thresholdFraction;
  }

  /**
   * Intake spin-up region: true while still below the magnitude threshold (uses
   * absolute values so negative intake RPM works).
   */
  public static boolean intakeBelowSpinupFraction(
      double currentRpm, double targetRpm, double thresholdFraction) {
    return Math.abs(currentRpm) < Math.abs(targetRpm) * thresholdFraction;
  }
}
