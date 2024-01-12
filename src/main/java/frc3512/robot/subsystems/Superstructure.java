package frc3512.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc3512.robot.Constants;

public class Superstructure extends SubsystemBase {

  // Subsystems
  public final Swerve swerve = new Swerve();

  // Joysticks
  private final CommandXboxController driverXbox =
      new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);

  // Xbox Axis Values
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  public Superstructure() {}

  public void configureBindings() {
    driverXbox.x().onTrue(new InstantCommand(() -> swerve.zeroGyro()));
  }

  public void configureAxisActions() {
    swerve.setDefaultCommand(
        swerve.drive(
            () -> -driverXbox.getRawAxis(translationAxis),
            () -> -driverXbox.getRawAxis(strafeAxis),
            () -> -driverXbox.getRawAxis(rotationAxis)));
  }

  public void setMotorBrake(boolean brake) {
    swerve.setMotorBrake(brake);
  }

  public Command getAuton() {
    // TODO: Eventually write an Autos class and replace this line with the auto chosen from the
    // AutonChooser
    return new InstantCommand();
  }
}
