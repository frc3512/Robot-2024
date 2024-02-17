package frc3512.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc3512.lib.util.CANSparkMaxUtil;
import frc3512.lib.util.CANSparkMaxUtil.Usage;
import frc3512.robot.Constants;

public class Elevator extends SubsystemBase {
  private CANSparkMax elevatorMotor = new CANSparkMax(13, MotorType.kBrushless);
  private DutyCycleEncoder elevatorEncoder = new DutyCycleEncoder(1);

  boolean bypassStop = false;

  public Elevator() {
    elevatorMotor.restoreFactoryDefaults();

    CANSparkMaxUtil.setCANSparkMaxBusUsage(elevatorMotor, Usage.kPositionOnly);

    elevatorMotor.setIdleMode(IdleMode.kBrake);
    elevatorMotor.setSmartCurrentLimit(Constants.ElevatorConstants.currentLimit);
    elevatorMotor.enableVoltageCompensation(10);
    elevatorMotor.setInverted(true);

    elevatorMotor.burnFlash();

    elevatorMotor.set(0);

    SmartDashboard.putNumber("Elevator Encoder", elevatorEncoder.getDistance());
  }

  public void moveElevator(double speed) {
    if (elevatorEncoder.getDistance() <= 0.49 && elevatorEncoder.getDistance() >= -1.55) {
      elevatorMotor.set(speed);
    } else if (elevatorEncoder.getDistance() >= 0.49 && speed > 0) {
      bypassStop = true;
      elevatorMotor.set(speed);
    } else if (elevatorEncoder.getDistance() <= -1.55 && speed < 0) {
      bypassStop = true;
      elevatorMotor.set(speed);
    }
  }

  public void stopElevator() {
    elevatorMotor.set(0);
  }

  @Override
  public void periodic() {
    if ((elevatorEncoder.getDistance() >= 0.49 || elevatorEncoder.getDistance() <= -1.55)
        && !bypassStop) {
      stopElevator();
    } else if (elevatorEncoder.getDistance() <= 0.49 && elevatorEncoder.getDistance() >= -1.55) {
      bypassStop = false;
    }
    SmartDashboard.putNumber("Elevator Encoder", elevatorEncoder.getDistance());
  }
}
