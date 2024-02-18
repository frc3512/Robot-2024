package frc3512.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc3512.lib.util.CANSparkMaxUtil;
import frc3512.lib.util.CANSparkMaxUtil.Usage;
import frc3512.robot.Constants;
import frc3512.robot.Constants.ElevatorConstants;

public class Elevator extends PIDSubsystem {
  private CANSparkMax elevatorMotor = new CANSparkMax(13, MotorType.kBrushless);
  private DutyCycleEncoder elevatorEncoder = new DutyCycleEncoder(1);

  boolean bypassStop = false;

  public Elevator() {
    super(new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD));
    getController().setTolerance(0.01);
    setSetpoint(ElevatorConstants.stowPosition);

    elevatorMotor.restoreFactoryDefaults();

    CANSparkMaxUtil.setCANSparkMaxBusUsage(elevatorMotor, Usage.kPositionOnly);

    elevatorMotor.setIdleMode(IdleMode.kBrake);
    elevatorMotor.setSmartCurrentLimit(Constants.ElevatorConstants.currentLimit);
    elevatorMotor.enableVoltageCompensation(10);
    elevatorMotor.setInverted(true);

    elevatorMotor.burnFlash();

    SmartDashboard.putNumber("Elevator Encoder", elevatorEncoder.getDistance());
    SmartDashboard.putNumber("Elevator PID Setpoint", getSetpoint());
  }

  @Override
  public void useOutput(double output, double setpoint) {
    elevatorMotor.setVoltage(output);
  }

  @Override
  public double getMeasurement() {
    return elevatorEncoder.getDistance();
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

  public void stowElevator() {
    setSetpoint(ElevatorConstants.stowPosition);
    enable();
  }

  public void outElevator() {
    setSetpoint(ElevatorConstants.outPosition);
    enable();
  }

  public void stopElevator() {
    elevatorMotor.set(0);
    disable();
  }

  @Override
  public void periodic() {
    super.periodic();
    if ((elevatorEncoder.getDistance() >= 0.49 || elevatorEncoder.getDistance() <= -1.55)
        && !bypassStop) {
      stopElevator();
    } else if (elevatorEncoder.getDistance() <= 0.49 && elevatorEncoder.getDistance() >= -1.55) {
      bypassStop = false;
    }
    SmartDashboard.putNumber("Elevator Encoder", elevatorEncoder.getDistance());
    SmartDashboard.putNumber("Elevator PID Setpoint", getSetpoint());
  }
}
