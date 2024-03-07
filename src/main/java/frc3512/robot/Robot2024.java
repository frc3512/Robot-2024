package frc3512.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc3512.robot.subsystems.Superstructure;

public class Robot2024 {
  private final Superstructure superstructure = new Superstructure();

  public Robot2024() {
    // Configure the trigger and axis bindings
    configureBindings();
    configureAxisActions();
  }

  private void configureBindings() {
    superstructure.configureBindings();
  }

  private void configureAxisActions() {
    superstructure.configureAxisActions();
  }

  public void setMotorBrake(boolean brake) {
    superstructure.setMotorBrake(brake);
  }

  public Command getAutonomousCommand() {
    return superstructure.getAuton();
  }
}
