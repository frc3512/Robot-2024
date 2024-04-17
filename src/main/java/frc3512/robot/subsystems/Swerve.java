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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc3512.lib.logging.SpartanEntryManager;
import frc3512.robot.Constants;
import java.io.File;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Swerve extends SubsystemBase {
  private final SwerveDrive swerve;
  private final PIDController turnController =
      new PIDController(
          Constants.SwerveConstants.turnControllerP,
          Constants.SwerveConstants.turnControllerI,
          Constants.SwerveConstants.turnControllerD);

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
      BooleanSupplier doAim,
      PhotonCamera camera) {
    return run(
        () -> {
          PhotonPipelineResult result = camera.getLatestResult();
          double yaw = 0.0;
          if (result.hasTargets() && doAim.getAsBoolean()) {
            List<PhotonTrackedTarget> targets = result.getTargets();
            for (int i = 0; i < targets.size(); i++) {
              if (targets.get(i).getFiducialId() == 7 || targets.get(i).getFiducialId() == 4) {
                yaw = targets.get(i).getYaw();
              }
            }
            drive(
                swerve.swerveController.getRawTargetSpeeds(
                    MathUtil.applyDeadband(
                        translationX.getAsDouble() * swerve.getMaximumVelocity(),
                        Constants.SwerveConstants.swerveDeadband),
                    MathUtil.applyDeadband(
                        translationY.getAsDouble() * swerve.getMaximumVelocity(),
                        Constants.SwerveConstants.swerveDeadband),
                    -turnController.calculate(yaw, 0)));
          } else {
            swerve.drive(
                new Translation2d(
                    MathUtil.applyDeadband(
                        translationX.getAsDouble() * swerve.getMaximumVelocity(),
                        Constants.SwerveConstants.swerveDeadband),
                    MathUtil.applyDeadband(
                        translationY.getAsDouble() * swerve.getMaximumVelocity(),
                        Constants.SwerveConstants.swerveDeadband)),
                MathUtil.applyDeadband(
                    angularRotationX.getAsDouble() * swerve.getMaximumAngularVelocity(),
                    Constants.SwerveConstants.swerveDeadband),
                true,
                false);
          }
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
