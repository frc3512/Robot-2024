package frc3512.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
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
  public final LED led = new LED();
  public final Swerve swerve = new Swerve();
  public final Vision vision = new Vision();
  public final Shootake shootake = new Shootake(led);
  public final Climber climber = new Climber();
  public final Arm arm = new Arm(swerve, vision);

  // Joysticks
  private final CommandXboxController driverXbox =
      new CommandXboxController(Constants.OperatorConstants.driverControllerPort);

  private final CommandJoystick appendageJoystick =
      new CommandJoystick(Constants.OperatorConstants.appendageControllerPort);

  // Xbox Axis Values
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  boolean xbox_shooting = false;

  public Superstructure() {
    autos = new Autos(this);
  }

  public void configureBindings() {
    // Reset Gyro
    driverXbox.x().onTrue(new InstantCommand(() -> swerve.zeroGyro()));
    driverXbox
        .a()
        .onTrue(
            new InstantCommand(
                () ->
                    swerve.resetOdometry(
                        new Pose2d(
                            new Translation2d(14.0, 5.50),
                            new Rotation2d(Units.degreesToRadians(0))))));
    
    if (xbox_shooting) {
      driverXbox.leftBumper().onTrue(new InstantCommand(() -> arm.setGoalFromRange(true)).andThen(new InstantCommand(() -> arm.enable())));
      driverXbox.leftBumper().onFalse(new InstantCommand(() -> arm.setGoalFromRange(false)));

      driverXbox.rightBumper().onTrue(new InstantCommand(() -> shootake.setShooter(true)));
      driverXbox.rightBumper().onFalse(shootSequence());

      driverXbox.b().onTrue(new InstantCommand(() -> shootake.want_to_intake = true));
      driverXbox.b().onFalse(new InstantCommand(() -> shootake.want_to_intake = false));
    } else {
      driverXbox.rightBumper().onTrue(new InstantCommand(() -> arm.setGoalFromRange(true)).andThen(new InstantCommand(() -> arm.enable())));
      driverXbox.rightBumper().onFalse(new InstantCommand(() -> arm.setGoalFromRange(false)));
    }

    appendageJoystick.button(1).onTrue(subsystemAmp());

    appendageJoystick.button(3).onTrue(
      new InstantCommand(()-> shootake.setIntake(false)).andThen(new InstantCommand(() -> shootake.setShooter(true))));
    appendageJoystick.button(3).onFalse(shootSequence());

    appendageJoystick.button(2).onTrue(subsystemCloseShot());

    appendageJoystick.button(4).onTrue(subsystemStow());

    appendageJoystick.button(5).onTrue(subsystemIntake());

    appendageJoystick.button(6).onTrue(new InstantCommand(() -> shootake.want_to_intake = true));
    appendageJoystick.button(6).onFalse(new InstantCommand(() -> shootake.want_to_intake = false));

    appendageJoystick.button(7).onTrue(new InstantCommand(() -> shootake.want_to_outtake = true));
    appendageJoystick.button(7).onFalse(new InstantCommand(() -> shootake.want_to_outtake = false));



    appendageJoystick.button(10).onTrue(subsystemFarShot());

    appendageJoystick.button(11).onTrue(subsystemTrapPositon());

    // Shootake
    appendageJoystick.button(6).onTrue(new InstantCommand(() -> shootake.setIntake(true)));
    appendageJoystick.button(6).onFalse(new InstantCommand(() -> shootake.setIntake(false)));
    appendageJoystick.button(7).onTrue(new InstantCommand(() -> shootake.setOuttake(true)));
    appendageJoystick.button(7).onFalse(new InstantCommand(() -> shootake.setOuttake(false)));
    //shootake.setIntake(appendageJoystick.button(6).getAsBoolean());
    //shootake.setOuttake(appendageJoystick.button(7).getAsBoolean());

    appendageJoystick.button(12).onTrue(new InstantCommand(() ->
      shootake.stopIntakeAndShooter()));

    // Climber Controls
    appendageJoystick
        .axisLessThan(Joystick.AxisType.kY.value, -0.5)
        .and(appendageJoystick.button(9))
        .onTrue(new InstantCommand(() -> climber.motorUp()));
    appendageJoystick
        .axisLessThan(Joystick.AxisType.kY.value, -0.5)
        .and(appendageJoystick.button(9))
        .onFalse(new InstantCommand(() -> climber.stopClimbers()));

    appendageJoystick
        .axisGreaterThan(Joystick.AxisType.kY.value, 0.5)
        .and(appendageJoystick.button(9)
        ).onTrue(new InstantCommand(() -> climber.motorDown()));
    appendageJoystick
    .axisGreaterThan(Joystick.AxisType.kY.value, 0.5)
    .and(appendageJoystick.button(9))
    .onFalse(new InstantCommand(() -> climber.stopClimbers()));

  }

  public void configureAxisActions() {
    swerve.setDefaultCommand(
        swerve.driveCommand(
            () ->
                MathUtil.applyDeadband(
                    driverXbox.getRawAxis(translationAxis),
                    Constants.SwerveConstants.swerveDeadband),
            () ->
                MathUtil.applyDeadband(
                    driverXbox.getRawAxis(strafeAxis), Constants.SwerveConstants.swerveDeadband),
            () ->
                MathUtil.applyDeadband(
                    driverXbox.getRawAxis(rotationAxis), Constants.SwerveConstants.swerveDeadband),
            () -> driverXbox.leftBumper().getAsBoolean(),
            vision));
  }

  public void setMotorBrake(boolean brake) {
    swerve.setMotorBrake(brake);
  }

  public Command getAuton() {
    return autos.getSelected();
  }

  public SequentialCommandGroup shootSequence() {
    return new InstantCommand(() -> shootake.setIntake(true))
        .andThen(new WaitCommand(.75))
        .andThen(new InstantCommand(() -> shootake.setIntake(false)))
        .andThen(new InstantCommand(() -> shootake.setOuttake(false)))
        .andThen(new InstantCommand(() -> shootake.setShooter(false)))
        .andThen(new InstantCommand(() -> arm.stowArm()));
  }

  public InstantCommand subsystemStow() {
    return new InstantCommand(() -> arm.stowArm());
  }

  public InstantCommand subsystemIntake() {
    return new InstantCommand(() -> arm.intakePos());
  }

  public InstantCommand subsystemAmp() {
    return new InstantCommand(() -> arm.ampShootingPos());
  }

  public InstantCommand subsystemFarShot() {
    return new InstantCommand(() -> arm.farShootingPos());
  }

  public InstantCommand subsystemCloseShot() {
    return new InstantCommand(() -> arm.closeShootingPos());
  }

  public InstantCommand subsystemTrapPositon() {
    return new InstantCommand(() -> arm.trapPositon());
  }

  public SequentialCommandGroup subsytemStopIntakeAndShooter() {
    return new InstantCommand(() -> shootake.setIntake(false))
        .andThen(new InstantCommand(() -> shootake.setOuttake(false)))
        .andThen(new InstantCommand(() -> shootake.setShooter(false)));
  }

  public InstantCommand subsystemAutoShot() {
    return new InstantCommand(() -> arm.autoShootingPos());
  }

  @Override
  public void periodic() {
    vision.periodic(swerve);
  }

  @Override
  public void simulationPeriodic() {
    // Update camera simulation
    vision.simulationPeriodic(swerve.getPose());

    var debugField = vision.getSimDebugField();
    debugField.getObject("EstimatedRobot").setPose(swerve.getPose());
  }
}
