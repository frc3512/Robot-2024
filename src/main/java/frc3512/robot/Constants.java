package frc3512.robot;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
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

    public static final double turnControllerTolerance = 1.0;

    public static final double maximumSpeed = Units.feetToMeters(19.0);
  }

  public static final class GeneralConstants {
    public static final boolean tuningMode = true;
    public static final boolean enablePoseEstimation = true;
    public static final RobotType robotType = RobotType.PROTO;
  }

  public static final class VisionConstants {
    public static final String visionName = "USB_GS_Camera";
    public static final String driverName = "HD_Web_Camera";
    public static final Transform3d robotToCam = new Transform3d();

    public static final Pair<Integer, Translation2d> blueSpeaker =
        new Pair<Integer, Translation2d>(7, new Translation2d(0.00, 5.55));
    public static final Pair<Integer, Translation2d> redSpeakerDistance =
        new Pair<Integer, Translation2d>(4, new Translation2d(15.64, 5.55));
    public static final Pair<Integer, Translation2d> redSpeakerAim =
        new Pair<Integer, Translation2d>(4, new Translation2d(16.70, 5.55));

    public static final Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(4.0, 4.0, 4.0);
    public static final Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);
  }

  public static final class ArmConstants {
    public static final int currentLimit = 40;
    public static final double speed = 0.1;

    public static final double kP = 26;
    public static final double kI = 0.05;
    public static final double kD = 0.05;

    public static final double stowPosition = 0.4;
    public static final double closeShootingPosition = 0.3;
    public static final double farShootingPosition = 0.48;
    public static final double autoShootingPosition = 0.4;
    public static final double ampPosition = 0.95;
    public static final double intakePosition = 0.19;
    public static final double trapPositon = 0.95;
  }

  public static final class ElevatorConstants {
    public static final int currentLimit = 40;
    public static final double speed = 0.1;

    public static final double kP = 6.0;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double stowPosition = 0.0;
    public static final double outPosition = -1.9;
    public static final double ampPosition = -0.7;
    public static final double kMaxVelocityRadPerSecond = 10.0;
    public static final double kMaxAccelerationRadPerSecSquared = 15.0;

    public static final int averageSampleSize = 10;
    public static final double distancePerPulse =
        (Math.PI * 2.0 * Units.inchesToMeters(1.751)) / 8192;
  }

  public static final class ShooterConstants {
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double bkP = 0;
    public static final double bkI = 0;
    public static final double bkD = 0;

    public static final int kShooterToleranceRPM = 10;
  }
}
