package frc3512.robot.auton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc3512.robot.subsystems.Superstructure;
import java.util.HashMap;

public class Autos {

  private final Superstructure superstructure;
  private final SendableChooser<Command> autonChooser;
  private final HashMap<String, Command> eventMap;

  public Autos(Superstructure superstructure) {
    this.superstructure = superstructure;

    eventMap = new HashMap<>();
    setMarkers();

    autonChooser = new SendableChooser<Command>();
    autonChooser.setDefaultOption("No-Op", new InstantCommand());
    autonChooser.addOption("Forward", foward());
    autonChooser.addOption("Right", right());

    SmartDashboard.putData("Auton Chooser", autonChooser);
  }

  private void setMarkers() {
    eventMap.put("Reset Gyro", resetGyro());
  }

  public Command resetGyro() {
    return new InstantCommand(() -> superstructure.swerve.zeroGyro());
  }

  public Command getSelected() {
    return autonChooser.getSelected();
  }

  public Command foward() {
    return AutoBuilder.followPath(PathPlannerPath.fromPathFile("Forward"));
  }

  public Command right() {
    return AutoBuilder.followPath(PathPlannerPath.fromPathFile("Right"));
  }
}
