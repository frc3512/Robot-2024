package frc3512.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
  }

  public void configureBindings() {
    driverXbox.x().onTrue(new InstantCommand(() -> swerve.zeroGyro()));

    // Shooter/intake controls
    appendageJoystick.button(3).onTrue(new InstantCommand(() -> shootake.intake()));
    appendageJoystick.button(3).onFalse(new InstantCommand(() -> shootake.stopIntakeOutake()));
    appendageJoystick.button(4).onTrue(new InstantCommand(() -> shootake.outake()));
    appendageJoystick.button(4).onFalse(new InstantCommand(() -> shootake.stopIntakeOutake()));
    appendageJoystick.button(11).onTrue(new InstantCommand(() -> shootake.shootClose()));
    appendageJoystick.button(11).onFalse(new InstantCommand(() -> shootake.stopShooting()));
    appendageJoystick.button(9).onTrue(new InstantCommand(() -> shootake.shootMedium()));
    appendageJoystick.button(9).onFalse(new InstantCommand(() -> shootake.stopShooting()));
    appendageJoystick.button(7).onTrue(new InstantCommand(() -> shootake.shootFar()));
    appendageJoystick.button(7).onFalse(new InstantCommand(() -> shootake.stopShooting()));
    appendageJoystick.button(2).onTrue(new InstantCommand( () -> arm.stopArm()));
    appendageJoystick.button(3).onTrue(new InstantCommand( () -> arm.moveArm(0.1)));
    appendageJoystick.button(4).onTrue(new InstantCommand( () -> arm.moveArm(-0.1)));
    appendageJoystick.button(5).onTrue(new InstantCommand( () -> elevator.moveElevator(0.1)));
    appendageJoystick.button(6).onTrue(new InstantCommand( () -> elevator.moveElevator(-0.1)));
    appendageJoystick.button(7).onTrue(new InstantCommand( () -> elevator.stopElevator()));
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
            vision.returnCamera(vision)));
  }

  public void setMotorBrake(boolean brake) {
    swerve.setMotorBrake(brake);
  }

  public Command getAuton() {
    return autos.getSelected();
  }
}
