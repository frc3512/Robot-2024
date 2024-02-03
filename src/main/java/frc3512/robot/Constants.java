package frc3512.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {

  public static enum RobotType {
    PROTO,
    COMP,
    DEFAULT
  }

  public static class OperatorConstants {
    public static final int driverControllerPort = 0;
  }

  public static final class SwerveConstants {
    public static final double WHEEL_LOCK_TIME = 10; // seconds

    public static final double swerveDeadband = 0.1;

    public static final double maximumSpeed = Units.feetToMeters(19.0);
  }

  public static final class GeneralConstants {
    public static final boolean tuningMode = true;

    public static final RobotType robotType = RobotType.COMP;
  }
}
