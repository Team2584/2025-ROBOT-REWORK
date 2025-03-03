package frc.robot;

import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.CONSTANTS.CONSTANTS_PORTS;
import frc.robot.subsystems.State;
import frc.robot.subsystems.State.RobotState;
import frc.robot.subsystems.swerve.Drivetrain;

public class RobotContainer {
  private final CommandXboxController joystick = new CommandXboxController(CONSTANTS_PORTS.CONTROLLER_PORT);
  private final Joystick buttonBoard = new Joystick(CONSTANTS_PORTS.BUTTON_BOARD_PORT);

  private final Drivetrain drivetrain = new Drivetrain();
  private final State state = new State(this);

  @NotLogged
  SendableChooser<Command> autoChooser = new SendableChooser<>();

  Pair<RobotState, Pose2d>[] SELECTED_AUTO_PREP_MAP;
  String SELECTED_AUTO_PREP_MAP_NAME = "none"; // only used for logging
  int AUTO_PREP_NUM = 0;

  public Drivetrain getDrivetrain() {
    return this.drivetrain;
  }

  public CommandXboxController getController() {
    return this.joystick;
  }

  public Joystick getButtonBoard() {
    return this.buttonBoard;
  }

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}