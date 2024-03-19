package frc3512.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc3512.lib.util.CANSparkMaxUtil;
import frc3512.lib.util.CANSparkMaxUtil.Usage;

public class Climber extends SubsystemBase {
  private CANSparkMax climberMotor1 =
      new CANSparkMax(17, CANSparkLowLevel.MotorType.kBrushless);
  private CANSparkMax climberMotor2 =
      new CANSparkMax(16, CANSparkLowLevel.MotorType.kBrushless);

  private PowerDistribution pdh = new PowerDistribution();

  public Climber() {
    pdh.setSwitchableChannel(false);

    climberMotor1.restoreFactoryDefaults();
    climberMotor2.restoreFactoryDefaults();

    CANSparkMaxUtil.setCANSparkMaxBusUsage(climberMotor1, Usage.kMinimal);
    CANSparkMaxUtil.setCANSparkMaxBusUsage(climberMotor2, Usage.kMinimal);

    climberMotor1.setIdleMode(IdleMode.kBrake);
    climberMotor2.setIdleMode(IdleMode.kBrake);

    climberMotor1.setSmartCurrentLimit(50);
    climberMotor2.setSmartCurrentLimit(50);

    climberMotor1.enableVoltageCompensation(10);
    climberMotor2.enableVoltageCompensation(10);

    climberMotor2.setInverted(true);

    climberMotor1.burnFlash();
    climberMotor2.burnFlash();
  }

  public void motorUp() {
    climberMotor1.set(0.3);
    climberMotor2.set(0.3);
  }

  public void motorDown() {
    pdh.setSwitchableChannel(false);
    climberMotor1.set(-0.8);
    climberMotor2.set(-0.8);
  }

  public void stopClimbers() {
    climberMotor1.set(0);
    climberMotor2.set(0);
  }

  public void releaseReaction() {
    pdh.setSwitchableChannel(true);
  }
}
