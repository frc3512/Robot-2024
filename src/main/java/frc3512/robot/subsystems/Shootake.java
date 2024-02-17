package frc3512.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc3512.lib.util.CANSparkMaxUtil;
import frc3512.lib.util.CANSparkMaxUtil.Usage;

public class Shootake extends SubsystemBase {
  private CANSparkMax intakeMotor = new CANSparkMax(10, CANSparkLowLevel.MotorType.kBrushless);
  private CANSparkMax topMotor = new CANSparkMax(12, CANSparkLowLevel.MotorType.kBrushless);
  private CANSparkMax bottomMotor = new CANSparkMax(11, CANSparkLowLevel.MotorType.kBrushless);

  private PIDController controller = new PIDController(0.01, 0, 0);

  private DigitalInput noteEnterBeamBreak = new DigitalInput(0);

  private RelativeEncoder topEncoder = topMotor.getEncoder();
  private RelativeEncoder bottomEncoder = bottomMotor.getEncoder();

  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.05, 0.0023);

  boolean shooting = false;

  public Shootake() {
    intakeMotor.restoreFactoryDefaults();
    topMotor.restoreFactoryDefaults();
    bottomMotor.restoreFactoryDefaults();

    CANSparkMaxUtil.setCANSparkMaxBusUsage(intakeMotor, Usage.kMinimal);
    CANSparkMaxUtil.setCANSparkMaxBusUsage(topMotor, Usage.kMinimal);
    CANSparkMaxUtil.setCANSparkMaxBusUsage(bottomMotor, Usage.kMinimal);

    intakeMotor.setIdleMode(IdleMode.kBrake);
    topMotor.setIdleMode(IdleMode.kCoast);
    bottomMotor.setIdleMode(IdleMode.kCoast);

    intakeMotor.setSmartCurrentLimit(40);
    topMotor.setSmartCurrentLimit(40);
    bottomMotor.setSmartCurrentLimit(40);

    intakeMotor.enableVoltageCompensation(10);
    topMotor.enableVoltageCompensation(10);
    bottomMotor.enableVoltageCompensation(10);

    topMotor.setInverted(false);
    bottomMotor.setInverted(false);
  }

  public void intake() {
    intakeMotor.set(-0.8);
  }

  public void outake() {
    intakeMotor.set(0.8);
  }

  public void stopIntakeOutake() {
    intakeMotor.set(0.0);
  }

  public void shootClose() {
    topMotor.setVoltage(feedforward.calculate(1000));
    bottomMotor.setVoltage(feedforward.calculate(1000));
    shooting = true;
  }

  public void shootMedium() {
    topMotor.setVoltage(feedforward.calculate(1500));
    bottomMotor.setVoltage(feedforward.calculate(1500));
    shooting = true;
  }

  public void shootFar() {
    topMotor.setVoltage(feedforward.calculate(2500));
    bottomMotor.setVoltage(feedforward.calculate(2500));
    shooting = true;
  }

  public void stopShooting() {
    topMotor.set(0.0);
    bottomMotor.set(0.0);
    shooting = false;
  }

  @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber("Top Motor Velocity", topEncoder.getVelocity());
    SmartDashboard.putNumber("Bottom Motor Velocity", bottomEncoder.getVelocity());
    if (!noteEnterBeamBreak.get() && shooting == false) {
      stopIntakeOutake();
    }
  }
}
