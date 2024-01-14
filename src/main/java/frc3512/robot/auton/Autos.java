package frc3512.robot.auton;

import java.util.HashMap;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc3512.robot.subsystems.Superstructure;

public class Autos {

    private final Superstructure superstructure;
    private final SendableChooser<Command> autonChooser;
    private final HashMap<String, Command> eventMap;

    public Autos(Superstructure superstructure) {
        this.superstructure = superstructure;

        eventMap = new HashMap<>();
        setMarkers();

        autonChooser = new SendableChooser<Command>();
        SmartDashboard.putData("Auton Chooser", autonChooser);
    }

    private void setMarkers() {}

    public Command getSelected() {
        return autonChooser.getSelected();
    }
}
