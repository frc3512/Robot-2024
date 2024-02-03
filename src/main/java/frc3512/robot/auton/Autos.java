package frc3512.robot.auton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc3512.robot.subsystems.Superstructure;
import frc3512.robot.subsystems.Swerve;
import java.util.HashMap;

public class Autos {

  private final Superstructure superstructure;
  private final Swerve swerve;
  private final SendableChooser<Command> autonChooser;
  private final HashMap<String, Command> eventMap;

  public Autos(Superstructure superstructure, Swerve swerve) {
    this.superstructure = superstructure;
    this.swerve = swerve;

    eventMap = new HashMap<>();
    setMarkers();

    NamedCommands.registerCommand("Reset Gyro", resetGyro());

    AutoBuilder.configureHolonomic(
        swerve::getPose, // Robot pose supplier
        swerve
            ::resetOdometry, // Method to reset odometry (will be called if your auto has a starting
        // pose)
        swerve::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        swerve::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in
            // your Constants class
            new PIDConstants(0.0020645, 0.0, 0.0), // Translation PID constants
            new PIDConstants(0.01, 0.0, 0.0), // Rotation PID constants
            3.5, // Max module speed, in m/s
            0.4, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig(
                false, false) // Default path replanning config. See the API for the options
            // here
            ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          // var alliance = DriverStation.getAlliance();
          // if (alliance.isPresent()) {
          //  return alliance.get() == DriverStation.Alliance.Blue;
          // }
          return false;
        },
        swerve // Reference to this subsystem to set requirements
        );

    autonChooser = new SendableChooser<Command>();
    autonChooser.setDefaultOption("No-Op", new InstantCommand());
    autonChooser.addOption("L Path", lPath());
    autonChooser.addOption("Straight", straight());
    autonChooser.addOption("Square", square());
    autonChooser.addOption("Spin", spin());

    SmartDashboard.putData("Auton Chooser", autonChooser);
  }

  private void setMarkers() {
    eventMap.put("Reset Gyro", resetGyro());
  }

  public Command resetGyro() {
    return new InstantCommand(() -> swerve.zeroGyro());
  }

  public Command getSelected() {
    swerve.zeroGyro();
    return autonChooser.getSelected();
  }

  public Command lPath() {
    // return AutoBuilder.buildAuto("Forward");
    return AutoBuilder.followPath(PathPlannerPath.fromPathFile("L path"));
  }

  public Command straight() {
    return AutoBuilder.buildAuto("Straight");
  }

  public Command square() {
    return AutoBuilder.followPath(PathPlannerPath.fromPathFile("Square"));
  }

  public Command spin() {
    return AutoBuilder.followPath(PathPlannerPath.fromPathFile("Spin Path"));
  }
}
