package frc3512.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc3512.lib.util.CANSparkMaxUtil;
import frc3512.lib.util.CANSparkMaxUtil.Usage;
import frc3512.robot.Constants;

public class Arm extends ProfiledPIDSubsystem {
  private CANSparkMax leftMotor = new CANSparkMax(14, MotorType.kBrushless);
  private CANSparkMax rightMotor = new CANSparkMax(15, MotorType.kBrushless);
  private DutyCycleEncoder armEncoder = new DutyCycleEncoder(3);

  boolean bypassStop = false;

  public Arm() {
    super(
        new ProfiledPIDController(
            Constants.ArmConstants.kP,
            Constants.ArmConstants.kI,
            Constants.ArmConstants.kD,
            new TrapezoidProfile.Constraints(7, 5)));
    getController().setTolerance(0.002);
    setGoal(Constants.ArmConstants.stowPosition);

    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();

    CANSparkMaxUtil.setCANSparkMaxBusUsage(leftMotor, Usage.kMinimal);
    CANSparkMaxUtil.setCANSparkMaxBusUsage(rightMotor, Usage.kPositionOnly);

    leftMotor.setSmartCurrentLimit(Constants.ArmConstants.currentLimit);
    rightMotor.setSmartCurrentLimit(Constants.ArmConstants.currentLimit);

    leftMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setIdleMode(IdleMode.kBrake);

    leftMotor.setInverted(true);

    rightMotor.burnFlash();
    leftMotor.burnFlash();

    SmartDashboard.putNumber(
        "Arm/Arm Encoder Distance Per Rotation", armEncoder.getDistancePerRotation());
    SmartDashboard.putNumber("Arm/Arm Encoder", armEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Arm/Arm PID Goal", getController().getGoal().position);
  }

  @Override
  public void useOutput(double output, State setpoint) {
    leftMotor.setVoltage(output);
    rightMotor.setVoltage(output);
  }

  @Override
  public double getMeasurement() {
    return armEncoder.getAbsolutePosition();
  }

  public void moveArm(double speed) {
    if (armEncoder.getAbsolutePosition() <= 0.96 && armEncoder.getAbsolutePosition() >= 0.145) {
      leftMotor.set(speed);
      bypassStop = false;
    } else if (armEncoder.getAbsolutePosition() >= 0.96 && speed < 0) {
      bypassStop = true;
      leftMotor.set(speed);
    } else if (armEncoder.getAbsolutePosition() <= 0.145 && speed > 0) {
      bypassStop = true;
      leftMotor.set(speed);
    }
  }

  public void stowArm() {
    SmartDashboard.putString("Arm/Arm Test", "Stow Pos");
    setGoal(Constants.ArmConstants.stowPosition);
    enable();
  }

  public void ampShootingPos() {
    SmartDashboard.putString("Arm/Arm Test", "Amp Pos");
    setGoal(Constants.ArmConstants.ampPosition);
    enable();
  }

  public void intakePos() {
    SmartDashboard.putString("Arm/Arm Test", "Intake Pos");
    setGoal(Constants.ArmConstants.intakePosition);
    enable();
  }

  public void closeShootingPos() {
    SmartDashboard.putString("Arm/Arm Test", "Close Shooting Pos");
    setGoal(Constants.ArmConstants.closeShootingPosition);
    enable();
  }

  public void farShootingPos() {
    SmartDashboard.putString("Arm/Arm Test", "Far Shooting Pos");
    setGoal(Constants.ArmConstants.farShootingPosition);
    enable();
  }

  public void autoShootingPos() {
    SmartDashboard.putString("Arm/Arm Test", "Auto Shooting Pos");
    setGoal(Constants.ArmConstants.autoShootingPosition);
    enable();
  }

  public void autoCloseShootingPos() {
    SmartDashboard.putString("Arm/Arm Test", "Auto Shooting Pos");
    setGoal(Constants.ArmConstants.autoCloseShootingPosition);
    enable();
  }

  public void trapPositon() {
    setGoal(Constants.ArmConstants.trapPositon);
    enable();
  }

  public void stopArm() {
    disable();
    leftMotor.set(0);
    rightMotor.set(0);
  }

  @Override
  public void periodic() {
    super.periodic();
    if ((armEncoder.getAbsolutePosition() >= .999 || armEncoder.getAbsolutePosition() <= 0.145)
        && !bypassStop) {
      stopArm();
    } else if (armEncoder.getAbsolutePosition() <= 0.999
        && armEncoder.getAbsolutePosition() >= 0.145) {
      bypassStop = false;
    }

    SmartDashboard.putNumber("Arm/Arm P Value", getController().getP());
    SmartDashboard.putNumber(
        "Arm/Arm Encoder Distance Per Rotation", armEncoder.getDistancePerRotation());
    SmartDashboard.putNumber("Arm/Arm Encoder", armEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Arm/Arm PID Goal", getController().getGoal().position);
  }
}
