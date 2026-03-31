package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  /** Spark encoder objects; meaningful odometry only if {@link frc.robot.Constants.DriveConstants#DRIVE_QUADRATURE_ENCODERS_WIRED}. */
  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;

  // private final EncoderSim leftEncoderSim;
  // private final EncoderSim rightEncoderSim;

  /** 2D field visualization for simulation and dashboards. */
  private final Field2d field2d = new Field2d();

  // Simple internal pose integrator used only in simulation when drive encoders are
  // not wired. This avoids touching real encoders but lets teams see motion on
  // the 2D field.
  private Pose2d simPose = new Pose2d();
  private double lastSimUpdateSeconds = Timer.getFPGATimestamp();
  private double lastSimXSpeed = 0.0;
  private double lastSimZRotation = 0.0;
  // Tunable constants for how fast the simulated robot moves at full stick.
  private static final double SIM_MAX_LINEAR_METERS_PER_SEC = 3.0;
  private static final double SIM_MAX_ANGULAR_RAD_PER_SEC = Math.PI; // ~180°/s at full turn

  private static final double METERS_PER_ROTATION = (Math.PI * WHEEL_DIAMETER_METERS) / GEAR_RATIO;

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

    // Encoder API: brushed CIM needs quadrature wired to the data port for real counts.
    leftEncoder = leftLeader.getEncoder();
    rightEncoder = rightLeader.getEncoder();

    // setup simulation
    // leftEncoderSim = new EncoderSim(leftEncoder);
    // rightEncoderSim = new EncoderSim(rightEncoder);
  }

  // ---------------------------------------------------------------------------
  // Periodic
  // ---------------------------------------------------------------------------

  @Override
  public void periodic() {
    // Publish the Field2d so Shuffleboard / Sim GUI can attach a 2D field widget.
    SmartDashboard.putData("Drive/Field", field2d);

    if (DRIVE_QUADRATURE_ENCODERS_WIRED) {
      SmartDashboard.putNumber("Drive/Left distance (m)", getLeftDistanceMeters());
      SmartDashboard.putNumber("Drive/Right distance (m)", getRightDistanceMeters());
      SmartDashboard.putNumber("Drive/Left velocity (m/s)", getLeftVelocityMetersPerSecond());
      SmartDashboard.putNumber("Drive/Right velocity (m/s)", getRightVelocityMetersPerSecond());

      // Simple pose estimate assuming straight-line motion and heading = 0.
      double x = getAverageDistanceMeters();
      field2d.setRobotPose(new Pose2d(x, 0.0, new Rotation2d()));
    } else if (RobotBase.isSimulation()) {
      // Sim-only pose integration based on last commanded arcade inputs. This does
      // not touch real encoders and is only meant for visualization.
      double now = Timer.getFPGATimestamp();
      double dt = now - lastSimUpdateSeconds;
      lastSimUpdateSeconds = now;
      if (dt > 0.0 && dt < 0.1) {
        double linear = lastSimXSpeed * SIM_MAX_LINEAR_METERS_PER_SEC;
        double angular = lastSimZRotation * SIM_MAX_ANGULAR_RAD_PER_SEC;

        double newHeading = simPose.getRotation().getRadians() + angular * dt;
        double dx = linear * dt * Math.cos(newHeading);
        double dy = linear * dt * Math.sin(newHeading);
        simPose =
            new Pose2d(
                simPose.getX() + dx, simPose.getY() + dy, new Rotation2d(newHeading));
      }
      field2d.setRobotPose(simPose);

      SmartDashboard.putString(
          "Drive/Odometry",
          "Sim-only pose (no real drive encoders wired)");
    } else {
      SmartDashboard.putString(
          "Drive/Odometry",
          "Off (set DRIVE_QUADRATURE_ENCODERS_WIRED when encoders are wired)");
    }
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
    lastSimXSpeed = xSpeed;
    lastSimZRotation = zRotation;
    drive.arcadeDrive(xSpeed, zRotation);
  }

  /**
   * Sets raw voltage on each side of the drivetrain.
   * Useful for autonomous routines that command voltage directly (e.g. characterization).
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
   * Returns right-side kvelocity in meters per second.
   */
  public double getRightVelocityMetersPerSecond() {
    return (rightEncoder.getVelocity() / 60.0) * METERS_PER_ROTATION;
  }

  /** Resets both drive encoders to zero (no-op if drive encoders are not wired). */
  public void resetEncoders() {
    if (!DRIVE_QUADRATURE_ENCODERS_WIRED) {
      return;
    }
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  /**
   * Small fast back-and-forth wiggle motion, used while the intake/shooter
   * command is active. Runs until interrupted.
   *
   * @param wiggleSpeed       forward/backward speed [-1, 1], small magnitude
   * @param halfPeriodSeconds time for each half of the wiggle cycle
   */
  public Command commandIntakeWiggle(double wiggleSpeed, double halfPeriodSeconds) {
    Command forward = this.run(() -> driveArcade(wiggleSpeed, 0)).withTimeout(halfPeriodSeconds);
    Command backward = this.run(() -> driveArcade(-wiggleSpeed, 0)).withTimeout(halfPeriodSeconds);
    return Commands.sequence(forward, backward)
        .repeatedly()
        .finallyDo(interrupted -> stop());
  }
}