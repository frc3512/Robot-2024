package frc3512.robot;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.geometry.Transform3d;
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

    public static final double maximumSpeed = Units.feetToMeters(19.0);
  }

  public static final class GeneralConstants {
    public static final boolean tuningMode = true;

    public static final RobotType robotType = RobotType.PROTO;
  }

  public static final class VisionConstants {
    public static final String cameraName = "USB_GS_Camera";
    public static final Transform3d robotToCam = new Transform3d();
  }

  public static final class ArmConstants {
    public static final int currentLimit = 40;
    public static final double speed = 0.1;

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double minAngle = 10;
    public static final double maxAngle = 60;
  }

  public static final class ElevatorConstants {
    public static final int currentLimit = 40;
    public static final double speed = 0.1;

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
  }
}
