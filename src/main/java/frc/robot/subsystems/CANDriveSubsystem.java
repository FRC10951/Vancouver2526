// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.DriveEncoderMath;
import frc.robot.util.SparkMaxFaultReporter;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import static frc.robot.Constants.DriveConstants.*;

/**
 * Tank (differential) drive subsystem using CAN SparkMax controllers.
 *
 * <p>
 * Supports both tank drive (independent left/right sticks) and arcade drive
 * (forward + rotation), making it easy to switch control styles in
 * {@code Drive.java}.
 */
public class CANDriveSubsystem extends SubsystemBase {

  // --- Motors ---
  private final SparkMax leftLeader;
  private final SparkMax leftFollower;
  private final SparkMax rightLeader;
  private final SparkMax rightFollower;

  private final DifferentialDrive drive;

  // --- Encoders ---
  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;

  // private final EncoderSim leftEncoderSim;
  // private final EncoderSim rightEncoderSim;

  private static final double METERS_PER_ROTATION =
      DriveEncoderMath.metersPerMotorRotation(WHEEL_DIAMETER_METERS, GEAR_RATIO);

  private final SparkMaxFaultReporter leftLeaderFaults =
      new SparkMaxFaultReporter("Drive/Left leader");
  private final SparkMaxFaultReporter leftFollowerFaults =
      new SparkMaxFaultReporter("Drive/Left follower");
  private final SparkMaxFaultReporter rightLeaderFaults =
      new SparkMaxFaultReporter("Drive/Right leader");
  private final SparkMaxFaultReporter rightFollowerFaults =
      new SparkMaxFaultReporter("Drive/Right follower");

  private DifferentialDrivetrainSim drivetrainSim;
  private SparkMaxSim leftLeaderSim;
  private SparkMaxSim rightLeaderSim;

  public CANDriveSubsystem() {
    // Create brushed motors for a KitBot-style CIM drivetrain
    leftLeader = new SparkMax(LEFT_LEADER_ID, MotorType.kBrushed);
    leftFollower = new SparkMax(LEFT_FOLLOWER_ID, MotorType.kBrushed);
    rightLeader = new SparkMax(RIGHT_LEADER_ID, MotorType.kBrushed);
    rightFollower = new SparkMax(RIGHT_FOLLOWER_ID, MotorType.kBrushed);

    drive = new DifferentialDrive(leftLeader, rightLeader);

    // Longer CAN timeout is fine here because configuration only runs once at init
    leftLeader.setCANTimeout(250);
    rightLeader.setCANTimeout(250);
    leftFollower.setCANTimeout(250);
    rightFollower.setCANTimeout(250);

    // Left leader: invert so that positive values drive both sides forward
    SparkMaxConfig leftLeaderConfig = new SparkMaxConfig();
    leftLeaderConfig.voltageCompensation(12);
    leftLeaderConfig.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);
    leftLeaderConfig.inverted(true);
    leftLeader.configure(leftLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Right leader: not inverted
    SparkMaxConfig rightLeaderConfig = new SparkMaxConfig();
    rightLeaderConfig.voltageCompensation(12);
    rightLeaderConfig.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);
    rightLeader.configure(rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Followers mirror their respective leaders
    SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
    leftFollowerConfig.follow(leftLeader);
    leftFollower.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();
    rightFollowerConfig.follow(rightLeader);
    rightFollower.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Built-in encoders
    leftEncoder = leftLeader.getEncoder();
    rightEncoder = rightLeader.getEncoder();

    if (RobotBase.isSimulation()) {
      drivetrainSim =
          new DifferentialDrivetrainSim(
              DCMotor.getCIM(2),
              GEAR_RATIO,
              Constants.SimulationConstants.DRIVEBASE_J_KG_M2,
              Constants.SimulationConstants.DRIVEBASE_MASS_KG,
              WHEEL_DIAMETER_METERS / 2.0,
              DRIVE_TRACK_WIDTH_METERS,
              null);
      leftLeaderSim = new SparkMaxSim(leftLeader, DCMotor.getCIM(2));
      rightLeaderSim = new SparkMaxSim(rightLeader, DCMotor.getCIM(2));
    }
  }

  /**
   * Desktop simulation: differential drive plant + {@link SparkMaxSim} on leaders so
   * encoder distances/velocities track motor voltages.
   */
  public void simulationPeriodic() {
    if (drivetrainSim == null) {
      return;
    }
    double dt = TimedRobot.kDefaultPeriod;
    double vbus = RobotController.getBatteryVoltage();
    double leftVolts = leftLeader.getAppliedOutput() * vbus;
    double rightVolts = rightLeader.getAppliedOutput() * vbus;
    drivetrainSim.setInputs(leftVolts, rightVolts);
    drivetrainSim.update(dt);
    double leftMotorRpm =
        DriveEncoderMath.wheelLinearVelocityToMotorRpm(
            drivetrainSim.getLeftVelocityMetersPerSecond(), WHEEL_DIAMETER_METERS, GEAR_RATIO);
    double rightMotorRpm =
        DriveEncoderMath.wheelLinearVelocityToMotorRpm(
            drivetrainSim.getRightVelocityMetersPerSecond(), WHEEL_DIAMETER_METERS, GEAR_RATIO);
    leftLeaderSim.iterate(leftMotorRpm, vbus, dt);
    rightLeaderSim.iterate(rightMotorRpm, vbus, dt);
  }

  // ---------------------------------------------------------------------------
  // Periodic
  // ---------------------------------------------------------------------------

  @Override
  public void periodic() {
    // Publish useful diagnostics to SmartDashboard
    SmartDashboard.putNumber("Left Distance (m)", getLeftDistanceMeters());
    SmartDashboard.putNumber("Right Distance (m)", getRightDistanceMeters());
    SmartDashboard.putNumber("Left Velocity (m/s)", getLeftVelocityMetersPerSecond());
    SmartDashboard.putNumber("Right Velocity (m/s)", getRightVelocityMetersPerSecond());

    leftLeaderFaults.reportPeriodic(leftLeader);
    leftFollowerFaults.reportPeriodic(leftFollower);
    rightLeaderFaults.reportPeriodic(rightLeader);
    rightFollowerFaults.reportPeriodic(rightFollower);
  }

  // ---------------------------------------------------------------------------
  // Drive methods
  // ---------------------------------------------------------------------------

  /**
   * Tank drive: each side is controlled independently.
   * Positive values drive each side forward.
   *
   * @param leftSpeed  Speed for the left side [-1, 1]
   * @param rightSpeed Speed for the right side [-1, 1]
   */
  public void driveTank(double leftSpeed, double rightSpeed) {
    drive.tankDrive(leftSpeed, rightSpeed);
  }

  /**
   * Arcade drive: one stick for forward/backward, one for rotation.
   *
   * @param xSpeed    Forward/backward speed [-1, 1]
   * @param zRotation Rotation rate [-1, 1]
   */
  public void driveArcade(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation);
  }

  /**
   * Sets raw voltage on each side of the drivetrain.
   * Useful for autonomous routines that need precise distance control.
   *
   * @param leftVolts  Voltage for the left side [-12, 12]
   * @param rightVolts Voltage for the right side [-12, 12]
   */
  public void driveVolts(double leftVolts, double rightVolts) {
    leftLeader.setVoltage(leftVolts);
    rightLeader.setVoltage(rightVolts);
    drive.feed(); // Prevent motor safety watchdog from cutting power
  }

  /** Stops all drivetrain motors immediately. */
  public void stop() {
    drive.stopMotor();
  }

  // ---------------------------------------------------------------------------
  // Encoder helpers
  // ---------------------------------------------------------------------------

  /**
   * Returns the distance traveled by the left side in meters.
   * Negated so that forward motion (left motor inverted) gives positive distance.
   */
  public double getLeftDistanceMeters() {
    return -leftEncoder.getPosition() * METERS_PER_ROTATION;
  }

  /**
   * Returns the distance traveled by the right side in meters.
   */
  public double getRightDistanceMeters() {
    return rightEncoder.getPosition() * METERS_PER_ROTATION;
  }

  /**
   * Returns the average distance traveled by both sides in meters.
   * Convenient for straight-line distance calculations in auto.
   */
  public double getAverageDistanceMeters() {
    return (getLeftDistanceMeters() + getRightDistanceMeters()) / 2.0;
  }

  /**
   * Returns left-side velocity in meters per second.
   * Negated so that forward motion gives positive velocity.
   */
  public double getLeftVelocityMetersPerSecond() {
    return -(leftEncoder.getVelocity() / 60.0) * METERS_PER_ROTATION;
  }

  /**
   * Returns right-side velocity in meters per second.
   */
  public double getRightVelocityMetersPerSecond() {
    return (rightEncoder.getVelocity() / 60.0) * METERS_PER_ROTATION;
  }

  /** Resets both drive encoders to zero. */
  public void resetEncoders() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  /**
   * One full wiggle: forward along X first, then reverse the same amount so net pose is
   * approximately unchanged (symmetric timing and magnitude).
   */
  private Command commandIntakeWiggleOneCycle(double wiggleSpeed, double halfPeriodSeconds) {
    Command first = this.run(() -> driveArcade(wiggleSpeed, 0)).withTimeout(halfPeriodSeconds);
    Command second = this.run(() -> driveArcade(-wiggleSpeed, 0)).withTimeout(halfPeriodSeconds);
    return Commands.sequence(first, second);
  }

  /**
   * Small fast forward-then-reverse wiggle, used while the intake/shooter command
   * is active. Each cycle returns to roughly the same pose. Runs until interrupted.
   *
   * @param wiggleSpeed       forward/backward speed [-1, 1], small magnitude
   * @param halfPeriodSeconds time for each half of the wiggle cycle
   */
  public Command commandIntakeWiggle(double wiggleSpeed, double halfPeriodSeconds) {
    return commandIntakeWiggleOneCycle(wiggleSpeed, halfPeriodSeconds)
        .repeatedly()
        .finallyDo(interrupted -> stop());
  }

  /**
   * Wiggle used while shooting: continuous normal motion with one longer
   * forward-then-reverse cycle (same speed, {@link frc.robot.Constants.DriveConstants#INTAKE_WIGGLE_HARD_SHAKE_HALF_PERIOD_MULTIPLIER}× half-period) every {@link frc.robot.Constants.DriveConstants#INTAKE_WIGGLE_HARD_SHAKE_INTERVAL_SECONDS}.
   */
  public Command commandIntakeWiggleWhileShooting() {
    double half = INTAKE_WIGGLE_HALF_PERIOD_SECONDS;
    double hardHalf = half * INTAKE_WIGGLE_HARD_SHAKE_HALF_PERIOD_MULTIPLIER;
    double hardCycleSeconds = 2.0 * hardHalf;
    double normalPhaseSeconds =
        Math.max(0.0, INTAKE_WIGGLE_HARD_SHAKE_INTERVAL_SECONDS - hardCycleSeconds);
    Command normalPhase =
        Commands.deadline(
            Commands.waitSeconds(normalPhaseSeconds),
            commandIntakeWiggle(INTAKE_WIGGLE_SPEED, half));
    Command hardOnce = commandIntakeWiggleOneCycle(INTAKE_WIGGLE_SPEED, hardHalf);
    return Commands.sequence(normalPhase, hardOnce)
        .repeatedly()
        .finallyDo(interrupted -> stop());
  }
}