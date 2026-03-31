package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

/**
 * Velocity control for a brushless motor with Neo/Spark encoder feedback in RPM.
 *
 * <p>Internally converts to rotations per second for {@link SimpleMotorFeedforward} (kV in
 * V·s/rotation). Combines feedforward with a full PID loop on measured velocity.
 *
 * <p>Spin-up uses open-loop max voltage; PID integral is reset during spin-up to avoid windup.
 */
public final class RpmPidVelocityController {

  private final PIDController pid;
  private final SimpleMotorFeedforward feedforward;
  private final double maxAbsVoltage;
  private final double spinupThresholdFraction;

  /**
   * @param kp Proportional gain (output volts per (rot/s) of error).
   * @param ki Integral gain.
   * @param kd Derivative gain (output volts per (rot/s²); often small or zero on noisy velocity).
   * @param ks Static feedforward (volts).
   * @param kv Velocity feedforward (volts per rotation per second).
   * @param ka Acceleration feedforward (volts per (rot/s²)); use 0 if unknown.
   * @param integratorAbsMax Absolute cap on the PID integrator (same units as WPILib PIDController).
   * @param maxAbsVoltage Absolute output voltage clamp (e.g. 12).
   * @param spinupThresholdFraction While {@code |current| < |target| * fraction}, use spin-up mode.
   */
  public RpmPidVelocityController(
      double kp,
      double ki,
      double kd,
      double ks,
      double kv,
      double ka,
      double integratorAbsMax,
      double maxAbsVoltage,
      double spinupThresholdFraction) {
    this.pid = new PIDController(kp, ki, kd);
    this.pid.setIntegratorRange(-integratorAbsMax, integratorAbsMax);
    this.feedforward = new SimpleMotorFeedforward(ks, kv, ka);
    this.maxAbsVoltage = maxAbsVoltage;
    this.spinupThresholdFraction = spinupThresholdFraction;
  }

  /**
   * @param currentRpm measured velocity (RPM)
   * @param targetRpm setpoint (RPM); sign matters for intake (negative = reverse)
   * @return motor voltage command, or {@link Double#NaN} if caller should use open-loop spin-up
   *     instead
   */
  public double calculateVoltage(double currentRpm, double targetRpm) {
    if (targetRpm == 0.0) {
      pid.reset();
      return 0.0;
    }

    double currentRps = currentRpm / 60.0;
    double targetRps = targetRpm / 60.0;

    boolean spinup =
        Math.abs(currentRpm) < Math.abs(targetRpm) * spinupThresholdFraction;
    if (spinup) {
      pid.reset();
      return Double.NaN;
    }

    // Steady-state: FF at target velocity; PID trims error.
    double uFf = feedforward.calculate(targetRps);
    double uPid = pid.calculate(currentRps, targetRps);
    return MathUtil.clamp(uFf + uPid, -maxAbsVoltage, maxAbsVoltage);
  }

  /** Call when the loop is disabled so integral does not carry over. */
  public void reset() {
    pid.reset();
  }
}
