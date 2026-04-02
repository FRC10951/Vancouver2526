// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

/**
 * Encoder / wheel kinematics for the drivetrain (unit-testable, no hardware).
 */
public final class DriveEncoderMath {
  private DriveEncoderMath() {}

  /** Meters traveled per motor revolution at the wheel (motor shaft to wheel). */
  public static double metersPerMotorRotation(double wheelDiameterMeters, double gearRatio) {
    return (Math.PI * wheelDiameterMeters) / gearRatio;
  }

  /**
   * Motor RPM corresponding to a given wheel linear speed (m/s), given wheel
   * circumference and motor-to-wheel gear ratio.
   */
  public static double wheelLinearVelocityToMotorRpm(
      double wheelLinearMps, double wheelDiameterMeters, double gearRatio) {
    double wheelRps = wheelLinearMps / (Math.PI * wheelDiameterMeters);
    return wheelRps * 60.0 * gearRatio;
  }
}
