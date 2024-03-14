package frc3512.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc3512.lib.util.ScoringUtil;
import frc3512.robot.Constants;
import frc3512.robot.Robot;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

public class Vision extends SubsystemBase {
  public PhotonCamera visionCamera = new PhotonCamera(Constants.VisionConstants.visionName);
  public PhotonCamera driverCamera = new PhotonCamera(Constants.VisionConstants.driverName);
  public PhotonPoseEstimator photonPoseEstimator;
  public AprilTagFieldLayout layout;
  private double lastEstTimestamp = 0;
  private Rotation2d diffHeading = new Rotation2d();

  // Simulation
  private PhotonCameraSim cameraSim;
  private VisionSystemSim visionSim;

  public Vision() {
    PhotonCamera.setVersionCheckEnabled(false);
    driverCamera.setDriverMode(true);

    layout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();

    // Create pose estimator
    photonPoseEstimator =
        new PhotonPoseEstimator(
            layout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            visionCamera,
            Constants.VisionConstants.robotToCam);

    photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    // ----- Simulation
    if (Robot.isSimulation()) {
      // Create the vision system simulation which handles cameras and targets on the field.
      visionSim = new VisionSystemSim("main");
      // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
      visionSim.addAprilTags(layout);
      // Create simulated camera properties. These can be set to mimic your actual camera.
      var cameraProp = new SimCameraProperties();
      cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
      cameraProp.setCalibError(0.35, 0.10);
      cameraProp.setFPS(15);
      cameraProp.setAvgLatencyMs(50);
      cameraProp.setLatencyStdDevMs(15);
      // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
      // targets.
      cameraSim = new PhotonCameraSim(visionCamera, cameraProp);
      // Add the simulated camera to view the targets on this simulated field.
      visionSim.addCamera(cameraSim, Constants.VisionConstants.robotToCam);

      cameraSim.enableDrawWireframe(true);
    }
  }

  /**
   * The latest estimated robot pose on the field from vision data. This may be empty. This should
   * only be called once per loop.
   *
   * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
   *     used for estimation.
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    var visionEst = photonPoseEstimator.update();
    double latestTimestamp = visionCamera.getLatestResult().getTimestampSeconds();
    boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
    if (Robot.isSimulation()) {
      visionEst.ifPresentOrElse(
          est ->
              getSimDebugField()
                  .getObject("VisionEstimation")
                  .setPose(est.estimatedPose.toPose2d()),
          () -> {
            if (newResult) getSimDebugField().getObject("VisionEstimation").setPoses();
          });
    }
    if (newResult) lastEstTimestamp = latestTimestamp;
    return visionEst;
  }

  /**
   * Return the latest result from the vision camera's pipeline
   *
   * @return Latest result reported to us by the vision camera
   */
  public PhotonPipelineResult getLatestResult() {
    return visionCamera.getLatestResult();
  }

  /**
   * The standard deviations of the estimated pose from {@link #getEstimatedGlobalPose()}, for use
   * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
   * This should only be used when there are targets visible.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   */
  public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
    var estStdDevs = Constants.VisionConstants.singleTagStdDevs;
    var targets = getLatestResult().getTargets();
    int numTags = 0;
    double avgDist = 0;
    for (var tgt : targets) {
      var tagPose = photonPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty()) continue;
      numTags++;
      avgDist +=
          tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }
    if (numTags == 0) return estStdDevs;
    avgDist /= numTags;
    // Decrease std devs if multiple targets are visible
    if (numTags > 1) estStdDevs = Constants.VisionConstants.multiTagStdDevs;
    // Increase std devs based on (average) distance
    if (numTags == 1 && avgDist > 4)
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

    return estStdDevs;
  }

  /** Returns if the camera has targets or not */
  public boolean hasTargets() {
    return getLatestResult().hasTargets();
  }

  /*
   * Return the distance to the provided target from the provided robot pose
   */
  public double getTargetDistance(
      Supplier<Pose2d> poseSupplier, Supplier<Pair<Integer, Translation2d>> targetSupplier) {
    return getTargetDistance(poseSupplier.get().getTranslation(), targetSupplier.get().getSecond());
  }

  /*
   * Return the distance to the provided target from the provided robot pose
   */
  public double getTargetDistance(Translation2d current, Translation2d target) {
    return current.getDistance(target);
  }

  /** Set the current heading difference */
  public void setDifferenceHeading(Rotation2d rotation2d) {
    diffHeading = rotation2d;
  }

  /** Return the current heading difference */
  public Rotation2d getDifferenceHeading() {
    return diffHeading;
  }

  public void periodic(Swerve swerve) {
    var prevPose = swerve.getPose();
    // Correct pose estimate with vision measurements
    var visionEst = getEstimatedGlobalPose();

    // Only consider adding measurements in if we have targets
    visionEst.ifPresent(
        est -> {
          var estPose = est.estimatedPose.toPose2d();

          // Change our trust in the measurement based on the tags we can see
          var estStdDevs = getEstimationStdDevs(estPose);

          // Make sure it's on the field and doesn't have an ambiguity above 0.2
          // Also only does it if we explicity tell it to do so
          boolean ambiguityCheck = getLatestResult().targets.get(0).getPoseAmbiguity() < 0.2;
          boolean fieldCheck =
              (estPose.getX() > 0.0 && estPose.getX() <= layout.getFieldLength())
                  && (estPose.getY() > 0.0 && estPose.getY() <= layout.getFieldLength());
          SmartDashboard.putBoolean("Diagnostics/Vision/fieldCheck", fieldCheck);
          SmartDashboard.putBoolean("Diagnostics/Vision/ambiguietyCheck", ambiguityCheck);
          if (ambiguityCheck) {
            swerve.addVisionMeasurement(
                est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
          }
        });

    setDifferenceHeading(
        new Rotation2d(
            swerve.getPose().getX() - prevPose.getX(), swerve.getPose().getY() - prevPose.getY()));

    SmartDashboard.putNumber(
        "Diagnostics/Vision/Distance",
        getTargetDistance(() -> swerve.getPose(), () -> ScoringUtil.provideDistancePose()));

    SmartDashboard.putBoolean("Diagnostics/Vision/visionEst", visionEst.isPresent());
  }

  public void simulationPeriodic(Pose2d robotSimPose) {
    visionSim.update(robotSimPose);
  }

  /** Reset pose history of the robot in the vision system simulation. */
  public void resetSimPose(Pose2d pose) {
    if (Robot.isSimulation()) visionSim.resetRobotPose(pose);
  }

  /** A Field2d for visualizing our robot and objects on the field. */
  public Field2d getSimDebugField() {
    if (!Robot.isSimulation()) return null;
    return visionSim.getDebugField();
  }
}
