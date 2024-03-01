package frc3512.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
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
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Swerve extends SubsystemBase {
  private final SwerveDrive swerve;
  private final Vision vision;
  private final PIDController turnController =
      new PIDController(
          Constants.SwerveConstants.turnControllerP,
          Constants.SwerveConstants.turnControllerI,
          Constants.SwerveConstants.turnControllerD);

  public Swerve(Vision vision) {
    this.vision = vision;

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
      } else if (Constants.GeneralConstants.robotType == Constants.RobotType.PROTO) {
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

    // Heading correction should only be used while controlling the robot via angle.
    swerve.setHeadingCorrection(false);

    turnController.enableContinuousInput(-180.0, 180.0);
    turnController.setTolerance(Constants.SwerveConstants.turnControllerTolerance);
  }

  /**
   * Command to drive the robot using translative values and heading as angular velocity.
   *
   * @param translationX Translation in the X direction.
   * @param translationY Translation in the Y direction.
   * @param angularRotationX Rotation of the robot to set
   * @return Drive command.
   */
  public Command driveCommand(
      DoubleSupplier translationX,
      DoubleSupplier translationY,
      DoubleSupplier angularRotationX,
      BooleanSupplier doAim) {
    return run(
        () -> {
          if (vision.hasTargets() && doAim.getAsBoolean()) {
            aimAtPoint(
                translationX,
                translationY,
                angularRotationX,
                () -> ScoringUtil.provideScoringPose().getSecond(),
                true,
                true);
          } else {
            driveWithGyroYaw(
                MathUtil.applyDeadband(
                    translationX.getAsDouble() * swerve.getMaximumVelocity(),
                    Constants.SwerveConstants.swerveDeadband),
                MathUtil.applyDeadband(
                    translationY.getAsDouble() * swerve.getMaximumVelocity(),
                    Constants.SwerveConstants.swerveDeadband),
                MathUtil.applyDeadband(
                    angularRotationX.getAsDouble() * swerve.getMaximumAngularVelocity(),
                    Constants.SwerveConstants.swerveDeadband));
          }
        });
  }

  /**
   * Drive the robot given a chassis field oriented velocity. Uses the gyro's yaw instead of the
   * odometry yaw Prevents the constant reset that occurs if you feed in vision data
   */
  public Command driveWithGyroYaw(
      double translationX, double translationY, double angularRotationX) {
    return run(
        () -> {
          ChassisSpeeds fieldOrientedVelocity =
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  translationX, translationY, angularRotationX, swerve.getYaw());
          drive(fieldOrientedVelocity);
        });
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity) {
    swerve.driveFieldOriented(velocity);
  }

  /**
   * Drive according to the chassis robot oriented velocity.
   *
   * @param velocity Robot oriented {@link ChassisSpeeds}
   */
  public void drive(ChassisSpeeds velocity) {
    swerve.drive(velocity);
  }

  /**
   * Get the swerve drive kinematics object.
   *
   * @return {@link SwerveDriveKinematics} of the swerve drive.
   */
  public SwerveDriveKinematics getKinematics() {
    return swerve.kinematics;
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
  }

  private boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }

  /**
   * This will zero (calibrate) the robot to assume the current position is facing forward
   *
   * <p>If red alliance rotate the robot 180 after the drviebase zero command
   */
  public void zeroGyroWithAlliance() {
    if (isRedAlliance()) {
      zeroGyro();
      // Set the pose 180 degrees
      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    } else {
      zeroGyro();
    }
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
      boolean velocityCorrection) {
    // Calculate our desired robot velocities
    double moveRequest = Math.hypot(xRequestSupplier.getAsDouble(), yRequestSupplier.getAsDouble());
    double moveDirection =
        Math.atan2(yRequestSupplier.getAsDouble(), xRequestSupplier.getAsDouble());

    // If we don't feed in anything, be able to drive normally and return
    if (pointTranslation2dSupplier.get() == null) {
      swerve.drive(
          new Translation2d(
              MathUtil.applyDeadband(
                  xRequestSupplier.getAsDouble() * swerve.getMaximumVelocity(),
                  Constants.SwerveConstants.swerveDeadband),
              MathUtil.applyDeadband(
                  yRequestSupplier.getAsDouble() * swerve.getMaximumVelocity(),
                  Constants.SwerveConstants.swerveDeadband)),
          MathUtil.applyDeadband(
              rotateRequestSupplier.getAsDouble() * swerve.getMaximumAngularVelocity(),
              Constants.SwerveConstants.swerveDeadband),
          true,
          false);
    }

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
            ? currPose.getRotation().plus(Rotation2d.fromRadians(Math.PI)).getDegrees()
            : currPose.getRotation().getDegrees();
    double output = turnController.calculate(rotation, adjustedAngle.getDegrees());

    SmartDashboard.putNumberArray(
        "Diagnostics/Vision/Aim Point",
        new double[] {aimPoint.getX(), aimPoint.getY(), aimPoint.getAngle().getDegrees()});

    driveWithGyroYaw(
        -moveRequest * Math.cos(moveDirection), -moveRequest * Math.sin(moveDirection), output);
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
      boolean velocityCorrection) {
    return runEnd(
        () -> {
          aimAtPoint(
              xRequestSupplier,
              yRequestSupplier,
              rotateRequestSupplier,
              pointTranslation2dSupplier,
              reversed,
              velocityCorrection);
        },
        () -> {
          turnController.setSetpoint(swerve.getYaw().getDegrees());
          turnController.reset();
        });
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
   * Set chassis speeds with closed-loop velocity control.
   *
   * @param chassisSpeeds Chassis Speeds to set.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    swerve.setChassisSpeeds(chassisSpeeds);
  }

  /**
   * Post the trajectory to the field.
   *
   * @param trajectory The trajectory to post.
   */
  public void postTrajectory(Trajectory trajectory) {
    swerve.postTrajectory(trajectory);
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
    return swerve.getYaw();
  }

  /**
   * Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which
   * direction. The other for the angle of the robot.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param headingX X joystick which controls the angle of the robot.
   * @param headingY Y joystick which controls the angle of the robot.
   * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(
      double xInput, double yInput, double headingX, double headingY) {
    xInput = Math.pow(xInput, 3);
    yInput = Math.pow(yInput, 3);
    return swerve.swerveController.getTargetSpeeds(
        xInput,
        yInput,
        headingX,
        headingY,
        getHeading().getRadians(),
        Constants.SwerveConstants.maximumSpeed);
  }

  /**
   * Get the chassis speeds based on controller input of 1 joystick and one angle.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param angle The angle in as a {@link Rotation2d}.
   * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
    xInput = Math.pow(xInput, 3);
    yInput = Math.pow(yInput, 3);
    return swerve.swerveController.getTargetSpeeds(
        xInput,
        yInput,
        angle.getRadians(),
        getHeading().getRadians(),
        Constants.SwerveConstants.maximumSpeed);
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity() {
    return swerve.getFieldVelocity();
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
   * Get the {@link SwerveController} in the swerve drive.
   *
   * @return {@link SwerveController} from the {@link SwerveDrive}.
   */
  public SwerveController getSwerveController() {
    return swerve.swerveController;
  }

  /**
   * Get the {@link SwerveDriveConfiguration} object.
   *
   * @return The {@link SwerveDriveConfiguration} fpr the current drive.
   */
  public SwerveDriveConfiguration getSwerveDriveConfiguration() {
    return swerve.swerveDriveConfiguration;
  }

  /** Lock the swerve drive to prevent it from moving. */
  public void lock() {
    swerve.lockPose();
  }

  /**
   * Gets the current pitch angle of the robot, as reported by the imu.
   *
   * @return The heading as a {@link Rotation2d} angle
   */
  public Rotation2d getPitch() {
    return swerve.getPitch();
  }

  /*
    Add in vision measurement into the
  */
  public void addVisionMeasurement(
      Pose2d pose, double timestamp, Matrix<N3, N1> visionMeasurementStdDevs) {
    swerve.addVisionMeasurement(pose, timestamp, visionMeasurementStdDevs);
  }
}
