package frc3512.robot.auton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc3512.robot.Constants;
import frc3512.robot.subsystems.Superstructure;
import frc3512.robot.subsystems.Swerve;

public class Autos {

  private final Superstructure superstructure;
  private final Swerve swerve;
  private final SendableChooser<Command> autonChooser;

  public Autos(Superstructure superstructure, Swerve swerve) {
    this.superstructure = superstructure;
    this.swerve = swerve;

    setMarkers();

    AutoBuilder.configureHolonomic(
        swerve::getPose,
        swerve::resetOdometry,
        swerve::getRobotVelocity,
        swerve::setChassisSpeeds,
        new HolonomicPathFollowerConfig(
            Constants.AutonConstants.TRANSLATION_PID,
            Constants.AutonConstants.ANGLE_PID,
            4.5,
            swerve.getSwerveDriveConfiguration().getDriveBaseRadiusMeters(),
            new ReplanningConfig()),
        () -> {
          var alliance = DriverStation.getAlliance();
          return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
        },
        swerve);

    autonChooser = new SendableChooser<Command>();
    autonChooser.setDefaultOption("No-op", new InstantCommand());

    // Add autos
    buildAuto("Straight");
    buildAuto("Circle");
    buildAuto("1 Note Amp Start Auto");
    buildAuto("2 Note Amp Start Auto");
    buildAuto("3 Note Amp Start Auto");
    buildAuto("2 Note Mid Start Auto");

    SmartDashboard.putData("Auton Chooser", autonChooser);
  }

  public Command getSelected() {
    return autonChooser.getSelected();
  }

  private void setMarkers() {
    NamedCommands.registerCommand(
        "Reset Gyro", new InstantCommand(() -> swerve.zeroGyroWithAlliance()));
    NamedCommands.registerCommand("Shoot", new InstantCommand(/* Shoot */ ));
    NamedCommands.registerCommand("Intake", new InstantCommand(/* Intake */));
  }

  /*
   * Builds the auto and adds it into the chooser for you.
   * Use this method to save some time plz. :)
   *
   * @param autoName  Name of the auto file
   */
  private void buildAuto(String autoName) {
    Command autoCommand = AutoBuilder.buildAuto(autoName);
    autonChooser.addOption(autoName, autoCommand);
  }
}
