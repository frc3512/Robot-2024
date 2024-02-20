package frc3512.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc3512.lib.util.CANSparkMaxUtil;
import frc3512.lib.util.CANSparkMaxUtil.Usage;
import frc3512.robot.Constants.ShooterConstants;

public class Shootake extends PIDSubsystem {
  private CANSparkMax intakeMotor = new CANSparkMax(10, CANSparkLowLevel.MotorType.kBrushless);
  private CANSparkMax topMotor = new CANSparkMax(12, CANSparkLowLevel.MotorType.kBrushless);
  private CANSparkMax bottomMotor = new CANSparkMax(11, CANSparkLowLevel.MotorType.kBrushless);

  private DigitalInput noteEnterBeamBreak = new DigitalInput(0);

  private RelativeEncoder topEncoder = topMotor.getEncoder();
  private RelativeEncoder bottomEncoder = bottomMotor.getEncoder();

  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0.0026);

  boolean shooting = false;
  boolean manual_intake = false;

  public Shootake() {

    super(new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD));
    getController().setTolerance(ShooterConstants.kShooterToleranceRPM);
    setSetpoint(0);

    intakeMotor.restoreFactoryDefaults();
    topMotor.restoreFactoryDefaults();
    bottomMotor.restoreFactoryDefaults();

    CANSparkMaxUtil.setCANSparkMaxBusUsage(intakeMotor, Usage.kMinimal);
    CANSparkMaxUtil.setCANSparkMaxBusUsage(topMotor, Usage.kVelocityOnly, true);
    CANSparkMaxUtil.setCANSparkMaxBusUsage(bottomMotor, Usage.kVelocityOnly, true);

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
    bottomMotor.follow(topMotor);

    topEncoder.setMeasurementPeriod(20);
    topEncoder.setAverageDepth(4);

    bottomEncoder.setMeasurementPeriod(20);
    bottomEncoder.setAverageDepth(4);

    intakeMotor.burnFlash();
    topMotor.burnFlash();
    bottomMotor.burnFlash();
  }

  public void intake() {
    intakeMotor.set(-1);
  }

  public void outake() {
    intakeMotor.set(1);
  }

  public void stopIntakeOutake() {
    intakeMotor.set(0.0);
  }

  public void shootClose() {
    // topMotor.setVoltage(feedforward.calculate(1000));
    // bottomMotor.setVoltage(feedforward.calculate(1000));
    setSetpoint(1000);
    enable();
    shooting = true;
  }

  public void shootMedium() {
    // topMotor.setVoltage(feedforward.calculate(1500));
    // bottomMotor.setVoltage(feedforward.calculate(1500));
    setSetpoint(1500);
    enable();
    shooting = true;
  }

  public void shootFar() {
    // topMotor.setVoltage(feedforward.calculate(2500));
    // bottomMotor.setVoltage(feedforward.calculate(2500));
    setSetpoint(2500);
    enable();
    shooting = true;
  }

  public void stopShooting() {
    disable();
    topMotor.set(0.0);
    bottomMotor.set(0.0);
    shooting = false;
  }

  public void overrideBeamBreak(String mode) {
    if (mode == "manual") {
      manual_intake = true;
    } else if (mode == "auto") {
      manual_intake = false;
    } else {
      manual_intake = false;
    }
  }

  @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber("Shooter/Top Motor Velocity", topEncoder.getVelocity());
    SmartDashboard.putNumber("Shooter/Bottom Motor Velocity", bottomEncoder.getVelocity());
    SmartDashboard.putNumber("Shooter/PID Setpoint", getSetpoint());
    // if ((!noteEnterBeamBreak.get() && shooting == false) && !manual_intake) {
    //   stopIntakeOutake();
    // }
  }

  @Override
  public void useOutput(double output, double setPoint) {
    topMotor.setVoltage(output + feedforward.calculate(setPoint));
  }

  @Override
  public double getMeasurement() {
    return topEncoder.getVelocity();
  }
}
