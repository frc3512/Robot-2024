package frc3512.lib.util;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc3512.robot.Constants;

public class ScoringUtil {

  /*
   * Return the right pair to score at depending on the alliance
   */
  public static Pair<Integer, Translation2d> provideScoringPose() {
    return DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Blue
        ? Constants.VisionConstants.blueSpeaker
        : Constants.VisionConstants.redSpeakerAim;
  }

  /*
   * Return the right pair to get the distance depending on alliance
   */
  public static Pair<Integer, Translation2d> provideDistancePose() {
    return DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Blue
        ? Constants.VisionConstants.blueSpeaker
        : Constants.VisionConstants.redSpeakerDistance;
  }
}
