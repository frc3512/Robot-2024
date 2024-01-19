package frc3512.robot;

import edu.wpi.first.math.util.Units;
import swervelib.math.SwerveMath;

public final class Constants {

  public static enum RobotType {
    PROTO,
    COMP,
    DEFAULT
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
  }

  public static final class SwerveConstants {
    public static final double WHEEL_LOCK_TIME = 10; // seconds

    public static final double swerveDeadband = 0.1;

    public static final double maximumSpeed = Units.feetToMeters(14.5);

    // Angle conversion factor is 360 / (GEAR RATIO * ENCODER RESOLUTION)
    //  In this case the gear ratio is 12.8 motor revolutions per wheel rotation.
    //  The encoder resolution per motor revolution is 1 per motor revolution.
    public static final double angleConversionFactor =
        SwerveMath.calculateDegreesPerSteeringRotation(12.8, 1);

    // Motor conversion factor is (PI * WHEEL DIAMETER IN METERS) / (GEAR RATIO * ENCODER
    // RESOLUTION).
    //  In this case the wheel diameter is 4 inches, which must be converted to meters to get
    // meters/second.
    //  The gear ratio is 6.75 motor revolutions per wheel rotation.
    //  The encoder resolution per motor revolution is 1 per motor revolution.
    public static final double driveConversionFactor =
        SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(4), 6.75, 1);
  }

  public static final class GeneralConstants {
    public static final boolean tuningMode = true;

    public static final RobotType robotType = RobotType.PROTO;
  }
}
