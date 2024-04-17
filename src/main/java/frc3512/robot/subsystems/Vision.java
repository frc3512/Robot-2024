package frc3512.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc3512.robot.Constants;
import java.util.Optional;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class Vision extends SubsystemBase {
  public PhotonCamera photonCamera = new PhotonCamera(Constants.VisionConstants.visionName);
  // public UsbCamera driverCamera = new UsbCamera("Driver Camera",
  // Constants.VisionConstants.driverName);
  public PhotonPoseEstimator photonPoseEstimator;
  public AprilTagFieldLayout atfl;
  Thread m_driverCamThread;

  public Vision() {
    m_driverCamThread =
        new Thread(
            () -> {
              UsbCamera driverCamera = CameraServer.startAutomaticCapture();

              driverCamera.setResolution(320, 200);

              CvSink cvSink = CameraServer.getVideo();
              CvSource outputStream = CameraServer.putVideo("Rectangle", 640, 400);

              Mat mat = new Mat();

              while (!Thread.interrupted()) {
                if (cvSink.grabFrame(mat) == 0) {
                  outputStream.notifyError(cvSink.getError());
                  continue;
                }
                Imgproc.rectangle(
                    mat, new Point(100, 100), new Point(400, 400), new Scalar(255, 255, 255), 5);
                outputStream.putFrame(mat);
              }
            });
    m_driverCamThread.setDaemon(true);
    m_driverCamThread.start();

    PhotonCamera.setVersionCheckEnabled(false);
    // photonCamera.setDriverMode(true);

    atfl = AprilTagFields.kDefaultField.loadAprilTagLayoutField();

    // Create pose estimator
    photonPoseEstimator =
        new PhotonPoseEstimator(
            atfl,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            photonCamera,
            Constants.VisionConstants.robotToCam);
  }

  /**
   * @param estimatedRobotPose The current best guess at robot pose
   * @return A pair of the fused camera observations to a single Pose2d on the field, and the time
   *     of the observation. Assumes a planar field and the robot is always firmly on the ground
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEstimator.update();
  }

  public PhotonCamera returnCamera() {
    return photonCamera;
  }
}
