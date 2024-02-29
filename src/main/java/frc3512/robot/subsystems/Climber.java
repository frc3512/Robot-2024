package frc3512.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc3512.lib.util.CANSparkMaxUtil;
import frc3512.lib.util.CANSparkMaxUtil.Usage;

public class Climber extends SubsystemBase {
    private static CANSparkMax climberMotor1= new CANSparkMax(
    17, CANSparkLowLevel.MotorType.kBrushless);
    private static CANSparkMax climberMotor2= new CANSparkMax(
    16, CANSparkLowLevel.MotorType.kBrushless);

public Climber() {
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

    climberMotor1.setInverted(true);

    climberMotor1.burnFlash();
    climberMotor2.burnFlash();
 }

public static void motorUp() {
    climberMotor1.set(0.4);
    climberMotor2.set(0.4);
 }
public static void motorDown() {
    climberMotor1.set(-0.8);
    climberMotor2.set(-0.8);
 }
public static void stopClimbers() {
    climberMotor1.set(0);
    climberMotor2.set(0);
 }
 
}