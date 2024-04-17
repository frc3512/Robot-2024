package frc3512.robot;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public final class Constants {

  public static enum RobotType {
    PROTO,
    COMP,
    DEFAULT
  }

  public static class OperatorConstants {
    public static final int driverControllerPort = 0;
    public static final int appendageControllerPort = 1;
  }

  public static final class AutonConstants {

    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);
  }

  public static final class SwerveConstants {
    public static final double WHEEL_LOCK_TIME = 10; // seconds

    public static final double swerveDeadband = 0.1;

    public static final double turnControllerP = 0.1;
    public static final double turnControllerI = 0.0;
    public static final double turnControllerD = 0.0;

    public static final double maximumSpeed = Units.feetToMeters(19.0);
  }

  public static final class GeneralConstants {
    public static final boolean tuningMode = true;
    public static final boolean enablePoseEstimation =
        false; // Change to true to enable pose estimation

    public static final RobotType robotType = RobotType.COMP;
  }

  public static final class VisionConstants {
    public static final String visionName = "USB_GS_Camera";
    public static final String driverName = "HD_Web_Camera";

    public static final Pair<Integer, Translation2d> blueSpeaker =
        new Pair<Integer, Translation2d>(7, new Translation2d(0.00, 5.55));
    public static final Pair<Integer, Translation2d> redSpeaker =
        new Pair<Integer, Translation2d>(4, new Translation2d(15.64, 5.55));

    public static final Transform3d robotToCam =
        new Transform3d(
            Units.inchesToMeters(-11.0),
            Units.inchesToMeters(7.0),
            Units.inchesToMeters(16.5),
            new Rotation3d(0.0, Units.degreesToRadians(105.0), Units.degreesToRadians(180.0)));

    public static final Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);
    public static final Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);
  }

  public static final class ArmConstants {
    public static final int currentLimit = 40;
    public static final double speed = 0.1;

    public static final double kP = 37;
    public static final double kI = 0;
    public static final double kD = 0.01;

    /* WORKING CONSTANTS
    public static final double kP = 32;
    public static final double kI = 0;
    public static final double kD = 0.025;
     */

    public static final double stowPosition = 0.295; // 0.295
    public static final double autoCloseShootingPosition = 0.3; // 0.3
    public static final double closeShootingPosition = 0.31; // 0.31
    public static final double farShootingPosition = 0.35; // was .375, then 0.35
    public static final double autoShootingPosition = 0.4; // 0.4
    public static final double ampPosition = 0.4; // 0.4
    public static final double intakePosition = 0.246; // 0.243
    public static final double trapPositon = 0.48; // 0.48
  }
}
