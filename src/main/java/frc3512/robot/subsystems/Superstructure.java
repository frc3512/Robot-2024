package frc3512.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc3512.robot.Constants;
import frc3512.robot.auton.Autos;

public class Superstructure extends SubsystemBase {

  // Autons
  private final Autos autos;

  // Subsystems
  public final Vision vision = new Vision();
  public final Swerve swerve = new Swerve(vision);
  public final Arm arm = new Arm();
  public final Elevator elevator = new Elevator();
  public final Shootake shootake = new Shootake();
  public final Climber climber = new Climber();

  // Joysticks
  private final CommandXboxController driverXbox =
      new CommandXboxController(Constants.OperatorConstants.driverControllerPort);

  private final CommandJoystick appendageJoystick =
      new CommandJoystick(Constants.OperatorConstants.appendageControllerPort);

  // Xbox Axis Values
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  public Superstructure() {
    autos = new Autos(this, swerve);
    SmartDashboard.putData(arm);
  }

  public void configureBindings() {
    driverXbox.x().onTrue(new InstantCommand(() -> swerve.zeroGyro()));

    appendageJoystick.button(1).onTrue(subsystemAmp());

    appendageJoystick.button(2).onTrue(subsystemCloseShot());

    appendageJoystick.button(3).onTrue(new InstantCommand(() -> shootake.shoot()));
    appendageJoystick.button(3).onFalse(shootSequence());

    appendageJoystick.button(4).onTrue(subsystemStow());

    appendageJoystick.button(5).onTrue(subsystemIntake());

    appendageJoystick.button(6).onTrue(new InstantCommand(() -> shootake.want_to_intake = true));
    appendageJoystick.button(6).onFalse(new InstantCommand(() -> shootake.want_to_intake = false));

    appendageJoystick.button(7).onTrue(new InstantCommand(() -> shootake.want_to_outtake = true));
    appendageJoystick.button(7).onFalse(new InstantCommand(() -> shootake.want_to_outtake = false));

    appendageJoystick.button(10).onTrue(subsystemFarShot());

    appendageJoystick.button(11).onTrue(subsystemTrapPositon());

    // appendageJoystick.button(11).onTrue(new InstantCommand(() -> Climber.motorUp()));
    // appendageJoystick.button(11).onFalse(new InstantCommand(() -> Climber.stopClimbers()));

    // appendageJoystick.button(12).onTrue(new InstantCommand(() -> Climber.motorDown()));
    // appendageJoystick.button(12).onFalse(new InstantCommand(() -> Climber.stopClimbers()));

    appendageJoystick
        .axisLessThan(Joystick.AxisType.kY.value, -0.5)
        .and(appendageJoystick.button(9))
        .onTrue(new InstantCommand(() -> Climber.motorUp()));
    appendageJoystick
        .axisLessThan(Joystick.AxisType.kY.value, -0.5)
        .and(appendageJoystick.button(9))
        .onFalse(new InstantCommand(() -> Climber.stopClimbers()));

    appendageJoystick
        .axisGreaterThan(Joystick.AxisType.kY.value, 0.5)
        .and(appendageJoystick.button(9))
        .onTrue(new InstantCommand(() -> Climber.motorDown()));
    appendageJoystick
        .axisGreaterThan(Joystick.AxisType.kY.value, 0.5)
        .and(appendageJoystick.button(9))
        .onFalse(new InstantCommand(() -> Climber.stopClimbers()));
  }

  public void configureAxisActions() {
    swerve.setDefaultCommand(
        swerve.driveCommand(
            () ->
                MathUtil.applyDeadband(
                    -driverXbox.getRawAxis(translationAxis),
                    Constants.SwerveConstants.swerveDeadband),
            () ->
                MathUtil.applyDeadband(
                    -driverXbox.getRawAxis(strafeAxis), Constants.SwerveConstants.swerveDeadband),
            () ->
                MathUtil.applyDeadband(
                    -driverXbox.getRawAxis(rotationAxis), Constants.SwerveConstants.swerveDeadband),
            () -> driverXbox.leftBumper().getAsBoolean()));
  }

  public void setMotorBrake(boolean brake) {
    swerve.setMotorBrake(brake);
  }

  public Command getAuton() {
    return autos.getSelected();
  }

  public SequentialCommandGroup shootSequence() {
    return new InstantCommand(() -> shootake.intake())
        .andThen(new WaitCommand(.75))
        .andThen(new InstantCommand(() -> shootake.stopIntakeOutake()))
        .andThen(new InstantCommand(() -> shootake.stopShooting()))
        .andThen(new InstantCommand(() -> arm.stowArm()))
        .andThen(new InstantCommand(() -> elevator.stowElevator()));
  }

  public SequentialCommandGroup subsystemStow() {
    return new InstantCommand(() -> arm.stowArm())
        .andThen(new WaitCommand(.5))
        .andThen(new InstantCommand(() -> elevator.stowElevator()));
  }

  public SequentialCommandGroup subsystemIntake() {
    return new InstantCommand(() -> elevator.outElevator())
        .andThen(new WaitCommand(.5))
        .andThen(new InstantCommand(() -> arm.intakePos()));
  }

  public SequentialCommandGroup subsystemAmp() {
    return new InstantCommand(() -> arm.ampShootingPos())
        .andThen(new WaitCommand(0.75))
        .andThen(new InstantCommand(() -> elevator.ampElevator()));
  }

  public SequentialCommandGroup subsystemFarShot() {
    return new InstantCommand(() -> arm.farShootingPos())
        .andThen(new InstantCommand(() -> elevator.outElevator()));
  }

  public SequentialCommandGroup subsystemCloseShot() {
    return new InstantCommand(() -> arm.closeShootingPos())
        .andThen(new InstantCommand(() -> elevator.outElevator()));
  }

  public SequentialCommandGroup subsystemTrapPositon() {
    return new InstantCommand(() -> arm.trapPositon())
        .andThen(new InstantCommand(() -> elevator.outElevator()));
  }

  public SequentialCommandGroup subsytemStopIntakeAndShooter() {
    return new InstantCommand(() -> shootake.stopIntakeOutake())
        .andThen(new InstantCommand(() -> shootake.stopShooting()));
  }

  public SequentialCommandGroup subsystemAutoShot() {
    return new InstantCommand(() -> arm.autoShootingPos())
        .andThen(new InstantCommand(() -> elevator.outElevator()));
  }

  @Override
  public void periodic() {
    vision.poseEstimationPeriodic(swerve);
  }
}
