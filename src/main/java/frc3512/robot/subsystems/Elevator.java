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

public class Elevator extends SubsystemBase{
    private CANSparkMax elevatorMotor = new CANSparkMax(13, MotorType.kBrushless);
    private DutyCycleEncoder elevatorEncoder = new DutyCycleEncoder(1);

    public Elevator() {
        elevatorMotor.restoreFactoryDefaults();

        CANSparkMaxUtil.setCANSparkMaxBusUsage(elevatorMotor, Usage.kPositionOnly);

        elevatorMotor.setIdleMode(IdleMode.kCoast);
        elevatorMotor.setSmartCurrentLimit(Constants.ElevatorConstants.currentLimit);
        elevatorMotor.enableVoltageCompensation(10);

        elevatorMotor.burnFlash();

        elevatorMotor.set(0);

        SmartDashboard.putNumber("Elevator Encoder", elevatorEncoder.getAbsolutePosition());
    }

    public void moveElevator(double speed) {
        elevatorMotor.set(speed);
    }

    public void stopElevator() {
        elevatorMotor.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Encoder", elevatorEncoder.getAbsolutePosition());
    }
}
