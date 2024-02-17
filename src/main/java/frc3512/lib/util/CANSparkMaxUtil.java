package frc3512.lib.util;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

/** Sets motor usage for a Spark Max motor controller. */
public class CANSparkMaxUtil {
  public enum Usage {
    kAll,
    kPositionOnly,
    kVelocityOnly,
    kMinimal
  };

  /**
   * This function allows reducing a Spark Max's CAN bus utilization by reducing the periodic status
   * frame period of nonessential frames from 20ms to 500ms.
   *
   * <p>See
   * https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
   * for a description of the status frames.
   *
   * @param motor The motor to adjust the status frame periods on.
   * @param usage The status frame feedack to enable. kAll is the default when a CANSparkMax is
   *     constructed.
   * @param enableFollowing Whether to enable motor following.
   */
  public static void setCANSparkMaxBusUsage(
      CANSparkMax motor, Usage usage, boolean enableFollowing) {

    if (enableFollowing) {
      motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 10);
    } else {
      motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 500);
    }

    if (usage == Usage.kAll) {
      motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 20);
      motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 20);
      motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 50);
      motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus4, 20);
      motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5, 50);
      motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus6, 50);
    } else if (usage == Usage.kPositionOnly) {
      motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 500);
      motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 20);
      motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 500);
      motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus4, 20);
      motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5, 50);
      motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus6, 500);
    } else if (usage == Usage.kVelocityOnly) {
      motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 20);
      motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 500);
      motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 500);
      motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus4, 20);
      motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5, 500);
      motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus6, 50);
    } else if (usage == Usage.kMinimal) {
      motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 500);
      motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 500);
      motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 500);
      motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus4, 500);
      motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5, 500);
      motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus6, 500);
    }
  }

  /**
   * This function allows reducing a Spark Max's CAN bus utilization by reducing the periodic status
   * frame period of nonessential frames from 20ms to 500ms.
   *
   * <p>See
   * https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
   * for a description of the status frames.
   *
   * @param motor The motor to adjust the status frame periods on.
   * @param usage The status frame feedack to enable. kAll is the default when a CANSparkMax is
   *     constructed.
   */
  public static void setCANSparkMaxBusUsage(CANSparkMax motor, Usage usage) {
    setCANSparkMaxBusUsage(motor, usage, false);
  }
}
