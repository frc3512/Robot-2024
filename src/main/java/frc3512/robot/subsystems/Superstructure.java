package frc3512.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc3512.robot.Constants;
import frc3512.robot.auton.Autos;
import java.util.List;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Superstructure extends SubsystemBase {
  // Autons
  private final Autos autos;

  // Subsystems
  public final LED led = new LED();
  public final Arm arm = new Arm();
  public final Swerve swerve = new Swerve();
  public final Vision vision = new Vision();
  public final Shootake shootake = new Shootake(led);
  public final Climber climber = new Climber();

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
    // Reset Gyro
    driverXbox.x().onTrue(new InstantCommand(() -> swerve.zeroGyro()));

    arm.m_useRange = (driverXbox.rightBumper().getAsBoolean());

    // Arm / Shooter Controls
    appendageJoystick.button(1).onTrue(subsystemAmp());

    appendageJoystick
        .button(3)
        .onTrue(
            new InstantCommand(() -> shootake.setIntake(false))
                .andThen(new InstantCommand(() -> shootake.setShooter(true))));
    appendageJoystick.button(3).onFalse(shootSequence());

    appendageJoystick.button(2).onTrue(subsystemCloseShot());

    appendageJoystick.button(4).onTrue(subsystemStow());

    appendageJoystick.button(5).onTrue(subsystemIntake());

    appendageJoystick.button(9).onTrue(new InstantCommand(() -> climber.releaseReaction(true)));
    appendageJoystick.button(9).onFalse(new InstantCommand(() -> climber.releaseReaction(false)));

    appendageJoystick.button(10).onTrue(subsystemFarShot());

    appendageJoystick.button(11).onTrue(subsystemTrapPositon());

    // Shootake
    appendageJoystick.button(6).onTrue(new InstantCommand(() -> shootake.setIntake(true)));
    appendageJoystick.button(6).onFalse(new InstantCommand(() -> shootake.setIntake(false)));
    appendageJoystick.button(7).onTrue(new InstantCommand(() -> shootake.setOuttake(true)));
    appendageJoystick.button(7).onFalse(new InstantCommand(() -> shootake.setOuttake(false)));
    // shootake.setIntake(appendageJoystick.button(6).getAsBoolean());
    // shootake.setOuttake(appendageJoystick.button(7).getAsBoolean());

    appendageJoystick.button(12).onTrue(new InstantCommand(() -> shootake.stopIntakeAndShooter()));

    // Climber Controls
    appendageJoystick
        .axisLessThan(Joystick.AxisType.kY.value, -0.5)
        .and(appendageJoystick.button(9))
        .onTrue(new InstantCommand(() -> climber.motorUp()));
    appendageJoystick
        .axisLessThan(Joystick.AxisType.kY.value, -0.5)
        .and(appendageJoystick.button(9))
        .onFalse(new InstantCommand(() -> climber.stopClimbers()));

    appendageJoystick
        .axisGreaterThan(Joystick.AxisType.kY.value, 0.5)
        .and(appendageJoystick.button(9))
        .onTrue(new InstantCommand(() -> climber.motorDown()));
    appendageJoystick
        .axisGreaterThan(Joystick.AxisType.kY.value, 0.5)
        .and(appendageJoystick.button(9))
        .onFalse(new InstantCommand(() -> climber.stopClimbers()));
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

  public void setMotorBrake(boolean brake) {
    swerve.setMotorBrake(brake);
  }

  public Command getAuton() {
    return autos.getSelected();
  }

  public SequentialCommandGroup shootSequence() {
    return new InstantCommand(() -> shootake.setIntake(true))
        .andThen(new WaitCommand(.75))
        .andThen(new InstantCommand(() -> shootake.stopIntakeAndShooter()))
        .andThen(new InstantCommand(() -> arm.stowArm()));
  }

  public InstantCommand subsystemStow() {
    return new InstantCommand(() -> arm.stowArm());
  }

  public InstantCommand subsystemIntake() {
    return new InstantCommand(() -> arm.intakePos());
  }

  public InstantCommand subsystemAmp() {
    return new InstantCommand(() -> arm.ampShootingPos());
  }

  public InstantCommand subsystemFarShot() {
    return new InstantCommand(() -> arm.farShootingPos());
  }

  public InstantCommand subsystemCloseShot() {
    return new InstantCommand(() -> arm.closeShootingPos());
    // this is done to conserve teleop shooting. will change once tuned
  }

  public InstantCommand subsystemTrapPositon() {
    return new InstantCommand(() -> arm.trapPositon());
  }

  public InstantCommand subsystemAutoShot() {
    return new InstantCommand(() -> arm.autoShootingPos());
  }

  @Override
  public void periodic() {
    PhotonPipelineResult result = vision.photonCamera.getLatestResult();
    List<PhotonTrackedTarget> targets = result.getTargets();
    double[] tags = new double[targets.size()];
    for (int i = 0; i < targets.size(); i++) {
      tags[i] = targets.get(i).getFiducialId();
    }
    SmartDashboard.putNumberArray("Diagnostics/Vision/Tag IDs", tags);
  }
}
