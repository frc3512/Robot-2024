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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc3512.robot.Constants;
import frc3512.robot.subsystems.Superstructure;
import frc3512.robot.subsystems.Swerve;
import org.photonvision.PhotonCamera;

@SuppressWarnings(
    "unused") // The superstructure is going to be used, but the linter isn't that smart (yet) XD
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
    buildAuto("1 Note Run");
    buildAuto("2 Note");
    buildAuto("3 Note");

    SmartDashboard.putData("Auton Chooser", autonChooser);
  }

  public Command getSelected() {
    return autonChooser.getSelected();
  }

  private void setMarkers() {
    // Swerve
    NamedCommands.registerCommand("Reset Gyro", new InstantCommand(() -> swerve.zeroGyro()));
    NamedCommands.registerCommand(
        "Motor Fix",
        new InstantCommand(
            () ->
                swerve.driveCommand(
                    () -> 0.0, () -> 0.0, () -> 0.0, () -> false, new PhotonCamera(null))));

    // Arm + Elevator
    NamedCommands.registerCommand("Stow", superstructure.subsystemStow());
    NamedCommands.registerCommand("Intake Position", superstructure.subsystemIntake());
    NamedCommands.registerCommand("Close Shooting", superstructure.subsystemCloseShot());
    NamedCommands.registerCommand(
        "Close Shooting Auto", new InstantCommand(() -> superstructure.arm.autoCloseShootingPos()));
    NamedCommands.registerCommand(
        "Auto Shooting", new InstantCommand(() -> superstructure.arm.autoShootingPos()));
    NamedCommands.registerCommand(
        "Far Shooting", new InstantCommand(() -> superstructure.arm.farShootingPos()));

    // Shootake
    NamedCommands.registerCommand(
        "Intake", new InstantCommand(() -> superstructure.shootake.setIntake(true)));
    NamedCommands.registerCommand(
        "Stop Shooting", new InstantCommand(() -> superstructure.shootake.setShooter(false)));
    NamedCommands.registerCommand(
        "Shooting Speed",
        new InstantCommand(() -> superstructure.shootake.setIntake(false))
            .andThen(new InstantCommand(() -> superstructure.shootake.setShooter(true))));
    NamedCommands.registerCommand(
        "Stop Intake/Shooter",
        new InstantCommand(() -> superstructure.shootake.stopIntakeAndShooter()));

    // Superstructure
    NamedCommands.registerCommand(
        "Shoot",
        (new InstantCommand(() -> superstructure.shootake.setShooter(true)))
            .andThen(new WaitCommand(2.5))
            .andThen(superstructure.shootSequence()));
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
