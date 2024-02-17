package frc3512.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc3512.lib.util.CANSparkMaxUtil;
import frc3512.lib.util.CANSparkMaxUtil.Usage;
import frc3512.robot.Constants;

public class Arm extends SubsystemBase {
    private CANSparkMax leftMotor = new CANSparkMax(14, MotorType.kBrushless);
    private CANSparkMax rightMotor = new CANSparkMax(15, MotorType.kBrushless);
    private DutyCycleEncoder armEncoder = new DutyCycleEncoder(5);

    public Arm() {
        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();

        CANSparkMaxUtil.setCANSparkMaxBusUsage(leftMotor, Usage.kMinimal);
        CANSparkMaxUtil.setCANSparkMaxBusUsage(rightMotor, Usage.kPositionOnly);

        leftMotor.setSmartCurrentLimit(Constants.ArmConstants.currentLimit);
        rightMotor.setSmartCurrentLimit(Constants.ArmConstants.currentLimit);

        leftMotor.setIdleMode(IdleMode.kCoast);
        rightMotor.setIdleMode(IdleMode.kCoast);

        rightMotor.setInverted(true);
        rightMotor.follow(leftMotor);

        rightMotor.burnFlash();
        leftMotor.burnFlash();

        leftMotor.set(0);

        SmartDashboard.putNumber("Arm Encoder Distance Per Rotation", armEncoder.getDistancePerRotation());
        SmartDashboard.putNumber("Arm Encoder", armEncoder.getAbsolutePosition());
    }

    public void moveArm(double speed) {
        leftMotor.set(speed);
    }

    public void stopArm() {
        leftMotor.set(0);
        rightMotor.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Encoder Distance Per Rotation", armEncoder.getDistancePerRotation());
        SmartDashboard.putNumber("Arm Encoder", armEncoder.getAbsolutePosition());
    }
}
