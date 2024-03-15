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
import frc3512.lib.util.ScoringUtil;
import frc3512.robot.Constants;
import frc3512.robot.subsystems.Superstructure;

public class Autos {

  private final Superstructure superstructure;
  private final SendableChooser<Command> autonChooser;

  public Autos(Superstructure superstructure) {
    this.superstructure = superstructure;

    setMarkers();

    AutoBuilder.configureHolonomic(
        superstructure.swerve::getPose,
        superstructure.swerve::resetOdometry,
        superstructure.swerve::getRobotVelocity,
        superstructure.swerve::setChassisSpeeds,
        new HolonomicPathFollowerConfig(
            Constants.AutonConstants.TRANSLATION_PID,
            Constants.AutonConstants.ANGLE_PID,
            4.5,
            superstructure.swerve.getSwerveDriveConfiguration().getDriveBaseRadiusMeters(),
            new ReplanningConfig()),
        () -> {
          var alliance = DriverStation.getAlliance();
          return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
        },
        superstructure.swerve);

    autonChooser = new SendableChooser<Command>();
    autonChooser.setDefaultOption("No-op", new InstantCommand());

    // Add autos
    buildAuto("1 Note Run");
    buildAuto("3 Note Fast");
    buildAuto("3 Note");

    SmartDashboard.putData("Auton Chooser", autonChooser);
  }

  public Command getSelected() {
    return autonChooser.getSelected();
  }

  private void setMarkers() {
    NamedCommands.registerCommand(
        "Reset Gyro", new InstantCommand(() -> superstructure.swerve.zeroGyro()));
    NamedCommands.registerCommand(
        "Motor Fix",
        new InstantCommand(
            () ->
                superstructure.swerve.driveCommand(() -> 0, () -> 0, () -> 0, () -> false, null)));
    NamedCommands.registerCommand(
        "Intake", new InstantCommand(() -> superstructure.shootake.want_to_intake = true));
    NamedCommands.registerCommand("Stow", superstructure.subsystemStow());
    NamedCommands.registerCommand("Intake Position", superstructure.subsystemIntake());
    NamedCommands.registerCommand("Close Shooting", superstructure.subsystemCloseShot());
    NamedCommands.registerCommand(
        "Stop Intake/Shooter", superstructure.subsytemStopIntakeAndShooter());
    NamedCommands.registerCommand(
        "Auto Shooting",
        (new InstantCommand(() -> superstructure.elevator.outElevator()))
            .andThen(new InstantCommand(() -> superstructure.arm.autoShootingPos())));
    NamedCommands.registerCommand(
        "Far Shooting",
        (new InstantCommand(() -> superstructure.elevator.outElevator()))
            .andThen(new InstantCommand(() -> superstructure.arm.farShootingPos())));
    NamedCommands.registerCommand(
        "Shoot",
        (new InstantCommand(() -> superstructure.shootake.shoot()))
            .andThen(new WaitCommand(2))
            .andThen(superstructure.shootSequence()));
    NamedCommands.registerCommand(
        "Stop Shooting", new InstantCommand(() -> superstructure.shootake.stopShooting()));
    NamedCommands.registerCommand(
        "Shooting Speed", new InstantCommand(() -> superstructure.shootake.shoot()));
    NamedCommands.registerCommand(
        "Aim at Speaker",
        superstructure.swerve.aimAtPointCommand(
            () -> 0,
            () -> 0,
            () -> 0,
            () -> ScoringUtil.provideScoringPose().getSecond(),
            true,
            true,
            superstructure.vision));
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
