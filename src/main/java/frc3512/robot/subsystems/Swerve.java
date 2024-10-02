package frc3512.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc3512.lib.logging.SpartanEntryManager;
import frc3512.lib.util.ScoringUtil;
import frc3512.robot.Constants;
import java.io.File;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Swerve extends SubsystemBase {
  private final SwerveDrive swerve;

  public Swerve() {
    if (SpartanEntryManager.isTuningMode()) {
      SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    } else {
      SwerveDriveTelemetry.verbosity = TelemetryVerbosity.LOW;
    }

    try {
      if (Constants.GeneralConstants.robotType == Constants.RobotType.COMP) {
        swerve =
            new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve/comp"))
                .createSwerveDrive(Constants.SwerveConstants.maximumSpeed);
      } else if (Constants.GeneralConstants.robotType == Constants.RobotType.COMP) {
        swerve =
            new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve/proto"))
                .createSwerveDrive(Constants.SwerveConstants.maximumSpeed);
      } else {
        // Default back to the comp bot
        swerve =
            new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve/comp"))
                .createSwerveDrive(Constants.SwerveConstants.maximumSpeed);
      }
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    // Disables cosine compensation for simulations since it causes discrepancies not seen in real
    // life.
    swerve.setCosineCompensator(!SwerveDriveTelemetry.isSimulation);
    swerve.swerveController.thetaController.setTolerance(2.5);
  }

  /**
   * Command to drive the robot using translative values and heading as angular velocity.
   *
   * @param translationX Translation in the X direction.
   * @param translationY Translation in the Y direction.
   * @param angularRotationX Rotation of the robot to set
   * @param doAim Button that allows the driver to aim
   * @param vision Vision subsystem passthrough
   * @return Drive command.
   */
  public Command driveCommand(
      DoubleSupplier translationX,
      DoubleSupplier translationY,
      DoubleSupplier angularRotationX,
      BooleanSupplier doAim,
      Vision vision) {
    return run(
        () -> {
          if (doAim.getAsBoolean() && vision.hasTargets()) {
            swerve.setHeadingCorrection(true);
            aimAtPoint(
                translationX,
                translationY,
                angularRotationX,
                () -> ScoringUtil.provideScoringPose().getSecond(),
                true,
                true,
                vision);
          } else {
            swerve.setHeadingCorrection(false);
            drive(
                MathUtil.applyDeadband(
                    translationX.getAsDouble() * swerve.getMaximumVelocity(),
                    Constants.SwerveConstants.swerveDeadband),
                MathUtil.applyDeadband(
                    translationY.getAsDouble() * swerve.getMaximumVelocity(),
                    Constants.SwerveConstants.swerveDeadband),
                MathUtil.applyDeadband(
                    angularRotationX.getAsDouble() * swerve.getMaximumVelocity(),
                    Constants.SwerveConstants.swerveDeadband),
                true);
          }
        });
  }

  /**
   * Aim the robot at a desired point on the field, while the driver is able to strafe
   *
   * @param xRequestSupplier X axis speed supplier [-1.0, +1.0]
   * @param yRequestSupplier Y axis speed supplier [-1.0, +1.0]
   * @param rotateRequestSupplier Rotate speed supplier (ONLY USED IF POINT IS NULL) [-1.0, +1.0]
   * @param pointSupplier Desired point supplier
   * @param reversed True to point rear of robot toward point
   * @param velocityCorrection True to compensate for robot's own velocity
   * @return Command that will aim at point while strafing
   */
  public Command aimAtPointCommand(
      DoubleSupplier xRequestSupplier,
      DoubleSupplier yRequestSupplier,
      DoubleSupplier rotateRequestSupplier,
      Supplier<Translation2d> pointTranslation2dSupplier,
      boolean reversed,
      boolean velocityCorrection,
      Vision vision) {
    return runEnd(
        () -> {
          aimAtPoint(
              xRequestSupplier,
              yRequestSupplier,
              rotateRequestSupplier,
              pointTranslation2dSupplier,
              reversed,
              velocityCorrection,
              vision);
        },
        () -> {
          swerve.swerveController.thetaController.setSetpoint(
              swerve.getOdometryHeading().getRadians());
          swerve.swerveController.thetaController.reset();
        });
  }

  /**
   * Aim the robot at a desired point on the field, while the driver is able to strafe
   *
   * @param xRequestSupplier X axis speed supplier [-1.0, +1.0]
   * @param yRequestSupplier Y axis speed supplier [-1.0, +1.0]
   * @param rotateRequestSupplier Rotate speed supplier (ONLY USED IF POINT IS NULL) [-1.0, +1.0]
   * @param pointSupplier Desired point supplier
   * @param reversed True to point rear of robot toward point
   * @param velocityCorrection True to compensate for robot's own velocity
   */
  public void aimAtPoint(
      DoubleSupplier xRequestSupplier,
      DoubleSupplier yRequestSupplier,
      DoubleSupplier rotateRequestSupplier,
      Supplier<Translation2d> pointTranslation2dSupplier,
      boolean reversed,
      boolean velocityCorrection,
      Vision vision) {

    // Turn on heading correction
    swerve.setHeadingCorrection(true);

    // Calculate our desired robot velocities
    double moveRequest = Math.hypot(xRequestSupplier.getAsDouble(), yRequestSupplier.getAsDouble());
    double moveDirection =
        Math.atan2(yRequestSupplier.getAsDouble(), xRequestSupplier.getAsDouble());

    // If we don't feed in anything, be able to drive normally and return
    if (pointTranslation2dSupplier.get() == null) {
      // Turn off heading correction (we don't need it)
      swerve.setHeadingCorrection(false);

      drive(
          MathUtil.applyDeadband(
              xRequestSupplier.getAsDouble() * swerve.getMaximumVelocity(),
              Constants.SwerveConstants.swerveDeadband),
          MathUtil.applyDeadband(
              yRequestSupplier.getAsDouble() * swerve.getMaximumVelocity(),
              Constants.SwerveConstants.swerveDeadband),
          MathUtil.applyDeadband(
              rotateRequestSupplier.getAsDouble() * swerve.getMaximumVelocity(),
              Constants.SwerveConstants.swerveDeadband),
          true);
    } else {
      Pose2d currPose = getPose();
      Vector2D robotVector =
          new Vector2D(
              moveRequest * vision.getDifferenceHeading().getCos(),
              moveRequest * vision.getDifferenceHeading().getSin());
      Translation2d aimPoint =
          pointTranslation2dSupplier
              .get()
              .minus(new Translation2d(robotVector.getX(), robotVector.getY()));
      Rotation2d targetAngle =
          new Rotation2d(
              pointTranslation2dSupplier.get().getX() - currPose.getX(),
              pointTranslation2dSupplier.get().getY() - currPose.getY());
      Vector2D targetVector =
          new Vector2D(
              currPose.getTranslation().getDistance(pointTranslation2dSupplier.get())
                  * targetAngle.getCos(),
              currPose.getTranslation().getDistance(pointTranslation2dSupplier.get())
                  * targetAngle.getSin());
      Vector2D parallelRobotVector =
          targetVector.scalarMultiply(
              robotVector.dotProduct(targetVector) / targetVector.getNormSq());
      Vector2D perpendicularRobotVector =
          robotVector.subtract(parallelRobotVector).scalarMultiply(velocityCorrection ? 0.3 : 0.0);
      Translation2d adjustedPoint =
          pointTranslation2dSupplier
              .get()
              .minus(
                  new Translation2d(
                      perpendicularRobotVector.getX(), perpendicularRobotVector.getY()));
      Rotation2d adjustedAngle =
          new Rotation2d(
              adjustedPoint.getX() - currPose.getX(), adjustedPoint.getY() - currPose.getY());

      double rotation =
          (reversed)
              ? currPose.getRotation().plus(Rotation2d.fromRadians(Math.PI)).getRadians()
              : currPose.getRotation().getRadians();
      double output =
          swerve.swerveController.thetaController.calculate(rotation, adjustedAngle.getRadians());

      SmartDashboard.putNumberArray(
          "Diagnostics/Vision/Aim Point", new double[] {aimPoint.getX(), aimPoint.getY()});

      drive(
          -moveRequest * Math.cos(moveDirection),
          -moveRequest * Math.sin(moveDirection),
          output,
          true);
    }
  }

  /** The primary method for controlling the drivebase. */
  public void drive(
      double translationX, double translationY, double rotation, boolean fieldRelative) {
    ChassisSpeeds velocity =
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translationX, translationY, rotation, swerve.getYaw())
            : new ChassisSpeeds(translationX, translationY, rotation);

    swerve.drive(velocity, false, new Translation2d());
  }

  /**
   * Set chassis speeds with closed-loop velocity control.
   *
   * @param chassisSpeeds Chassis Speeds to set.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    swerve.setChassisSpeeds(chassisSpeeds);
  }

  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when
   * calling this method. However, if either gyro angle or module position is reset, this must be
   * called in order for odometry to keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose) {
    swerve.resetOdometry(initialHolonomicPose);
    // swerve.setGyroOffset(new Rotation3d(0.0, 0.0, 180.0));
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
   */
  public void zeroGyro() {
    swerve.zeroGyro();
  }

  /**
   * Sets the drive motors to brake/coast mode.
   *
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake) {
    swerve.setMotorIdleMode(brake);
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the imu. CCW positive, not wrapped.
   *
   * @return The yaw angle
   */
  public Rotation2d getHeading() {
    return swerve.getOdometryHeading();
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose() {
    return swerve.getPose();
  }

  /**
   * Returns the YAGSL swerve drive object
   *
   * @return this subsystem's YAGSL swerve drive instance
   */
  public SwerveDrive getSwerveDriveObject() {
    return swerve;
  }

  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity() {
    return swerve.getRobotVelocity();
  }

  /**
   * Get the {@link SwerveDriveConfiguration} object.
   *
   * @return The {@link SwerveDriveConfiguration} fpr the current drive.
   */
  public SwerveDriveConfiguration getSwerveDriveConfiguration() {
    return swerve.swerveDriveConfiguration;
  }

  /*
    Add in vision measurement into the
  */
  public void addVisionMeasurement(
      Pose2d pose, double timestamp, Matrix<N3, N1> visionMeasurementStdDevs) {
    swerve.addVisionMeasurement(pose, timestamp, visionMeasurementStdDevs);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(
        "Swerve/thetaController setpoint", swerve.swerveController.thetaController.getSetpoint());
    SmartDashboard.putNumber("Swerve/yaw", swerve.getYaw().getRadians());
  }
}
