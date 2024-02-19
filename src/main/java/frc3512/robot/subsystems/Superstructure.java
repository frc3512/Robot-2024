package frc3512.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc3512.lib.util.ScoringUtil;
import frc3512.robot.Constants;
import frc3512.robot.auton.Autos;

public class Superstructure extends SubsystemBase {

  // Autons
  private final Autos autos;

  // Subsystems
  public final Swerve swerve = new Swerve();
  public final Vision vision = new Vision();
  public final Shootake shootake = new Shootake();
  public final Arm arm = new Arm();
  public final Elevator elevator = new Elevator();

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

    // Shooter/intake controls
    appendageJoystick.button(3).onTrue(new InstantCommand(() -> shootake.intake()));
    appendageJoystick.button(3).onFalse(new InstantCommand(() -> shootake.stopIntakeOutake()));

    appendageJoystick.button(7).onTrue(new InstantCommand(() -> shootake.outake()));
    appendageJoystick.button(7).onFalse(new InstantCommand(() -> shootake.stopIntakeOutake()));

    appendageJoystick.button(12).onTrue(new InstantCommand(() -> shootake.shootClose()));
    appendageJoystick.button(12).onFalse(new InstantCommand(() -> shootake.stopShooting()));

    appendageJoystick.button(11).onTrue(new InstantCommand(() -> shootake.shootMedium()));
    appendageJoystick.button(11).onFalse(new InstantCommand(() -> shootake.stopShooting()));

    appendageJoystick.button(10).onTrue(new InstantCommand(() -> shootake.shootFar()));
    appendageJoystick.button(10).onFalse(new InstantCommand(() -> shootake.stopShooting()));

    appendageJoystick
        .button(9)
        .whileFalse(new InstantCommand(() -> shootake.overrideBeamBreak("auto")));
    appendageJoystick
        .button(9)
        .whileTrue(new InstantCommand(() -> shootake.overrideBeamBreak("manual")));

    appendageJoystick.button(1).onTrue(new InstantCommand(() -> elevator.stowElevator()));
    appendageJoystick.button(2).onTrue(new InstantCommand(() -> elevator.outElevator()));

    appendageJoystick.button(4).onTrue(new InstantCommand(() -> arm.ampShootingPos())
      .andThen(new WaitCommand(1.5))
      .andThen(new InstantCommand(() -> elevator.outElevator())));

    appendageJoystick.button(5).onTrue(new InstantCommand(() -> arm.stowArm())
      .andThen(new WaitCommand(1.5))
      .andThen(new InstantCommand(() -> elevator.stowElevator())));
    // appendageJoystick.button(6).onTrue(new InstantCommand(() -> arm.intakePos()));

    appendageJoystick
        .button(6)
        .onTrue(
            new InstantCommand(() -> elevator.outElevator())
                .andThen(new WaitCommand(1.5))
                .andThen(new InstantCommand(() -> arm.intakePos())));
    /*appendageJoystick
        .button(7)
        .onTrue(
            new InstantCommand(
                () -> {
                  shootake.stopShooting();
                  arm.stopArm();
                }));

    appendageJoystick
        .button(1)
        .onTrue(
            new InstantCommand(
                () -> {
                  // arm.ampShootingPos();
                  elevator.stowElevator();
                }));
    appendageJoystick
        .button(4)
        .onTrue(
            new InstantCommand(
                () -> {
                  // arm.stowArm();
                  elevator.stowElevator();
                }));
    appendageJoystick
        .button(2)
        .onTrue(
            new InstantCommand(
                () -> {
                  // arm.intakePos();
                  elevator.outElevator();
                }));
    appendageJoystick.button(5).onTrue(new InstantCommand(() -> elevator.outElevator()));*/
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
            () -> driverXbox.leftBumper().getAsBoolean(),
            vision.returnCamera()));
  }

  private void poseEstimationPeriodic() {
    if (Constants.GeneralConstants.enablePoseEstimation) {
      // Correct pose estimate with vision measurements
      var visionEst = vision.getEstimatedGlobalPose();
      visionEst.ifPresent(
          est -> {
            var estPose = est.estimatedPose.toPose2d();
            // Change our trust in the measurement based on the tags we can see
            var estStdDevs = vision.getEstimationStdDevs(estPose);

            swerve.addVisionMeasurement(
                est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
          });
    }
  }

  public Measure<Distance> getTargetDistance() {
    return vision.getTargetDistance(swerve::getPose, () -> ScoringUtil.provideScoringPose());
  }

  public void setMotorBrake(boolean brake) {
    swerve.setMotorBrake(brake);
  }

  public Command getAuton() {
    return autos.getSelected();
  }

  @Override
  public void periodic() {
    poseEstimationPeriodic();
    SmartDashboard.putNumber(
        "Diagnostics/Vision/Vision Distance", getTargetDistance().baseUnitMagnitude());
  }
}
