package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.CONSTANTS.CONSTANTS_ELEVATOR;
import frc.robot.CONSTANTS.CONSTANTS_PORTS;
import frc.robot.commands.DriveTeleop;
import frc.robot.commands.zero.Zero_Elevator;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Ramp;
import frc.robot.subsystems.State;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.State.RobotState;
import frc.robot.subsystems.swerve.Drivetrain;

@Logged
public class RobotContainer {
  @NotLogged
  private final CommandXboxController controller = new CommandXboxController(CONSTANTS_PORTS.CONTROLLER_PORT);
  @NotLogged
  private final Joystick buttonBoard = new Joystick(CONSTANTS_PORTS.BUTTON_BOARD_PORT);

  private final State state = new State(this);
  private final Drivetrain drivetrain = new Drivetrain();
  private final Elevator elevator = new Elevator();
  private final Climber climber = new Climber();
  private final Ramp ramp = new Ramp();
  private final Wrist wrist = new Wrist();

  @NotLogged
  SendableChooser<Command> autoChooser = new SendableChooser<>();

  Command TRY_NONE = Commands.deferredProxy(() -> state.tryState(RobotState.NONE));
  Command TRY_CORAL_ZERO = Commands.deferredProxy(() -> state.tryState(RobotState.PREP_CORAL_ZERO));
  Command TRY_CORAL_L1 = Commands.deferredProxy(() -> state.tryState(RobotState.PREP_CORAL_L1));
  Command TRY_CORAL_L2 = Commands.deferredProxy(() -> state.tryState(RobotState.PREP_CORAL_L2));
  Command TRY_CORAL_L3 = Commands.deferredProxy(() -> state.tryState(RobotState.PREP_CORAL_L3));
  Command TRY_CORAL_L4 = Commands.deferredProxy(() -> state.tryState(RobotState.PREP_CORAL_L4));

  @NotLogged
  Pair<RobotState, Pose2d>[] SELECTED_AUTO_PREP_MAP;
  String SELECTED_AUTO_PREP_MAP_NAME = "none"; // only used for logging
  int AUTO_PREP_NUM = 0;

  @NotLogged
  public CommandXboxController getController() {
    return this.controller;
  }

  @NotLogged
  public Joystick getButtonBoard() {
    return this.buttonBoard;
  }

  public State getState() {
    return this.state;
  }

  public Drivetrain getDrivetrain() {
    return this.drivetrain;
  }

  public Elevator getElevator() {
    return this.elevator;
  }

  public Climber getClimber() {
    return this.climber;
  }

  public Ramp getRamp() {
    return this.ramp;
  }

  public Wrist getWrist() {
    return this.wrist;
  }

  // TODO: add other subsystems to this command
  Command zeroSubsystems = new ParallelCommandGroup(
      new Zero_Elevator(this).withTimeout(CONSTANTS_ELEVATOR.ZEROING_TIMEOUT.in(Units.Seconds)))
      .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming).withName("ZeroSubsystems");

  // Map buttons to trigger variables
  @NotLogged
  private final JoystickButton redL4 = new JoystickButton(getButtonBoard(), 1);
  @NotLogged
  private final JoystickButton redL3 = new JoystickButton(getButtonBoard(), 2);
  @NotLogged
  private final JoystickButton redL2 = new JoystickButton(getButtonBoard(), 3);
  @NotLogged
  private final JoystickButton redL1 = new JoystickButton(getButtonBoard(), 4);

  @NotLogged
  private final JoystickButton blue4 = new JoystickButton(getButtonBoard(), 5);
  @NotLogged
  private final JoystickButton blue3 = new JoystickButton(getButtonBoard(), 6);
  @NotLogged
  private final JoystickButton blue2 = new JoystickButton(getButtonBoard(), 7);
  @NotLogged
  private final JoystickButton blue1 = new JoystickButton(getButtonBoard(), 8);

  // Buttons
  public Trigger slowModeTrigger = new Trigger(() -> getController().y().getAsBoolean());
  public Trigger leftReefTrigger = new Trigger(() -> getController().x().getAsBoolean());
  public Trigger rightReefTrigger = new Trigger(() -> getController().b().getAsBoolean());

  public Trigger rightCoralStationTrigger = new Trigger(() -> blue4.getAsBoolean());
  public Trigger leftCoralStationTrigger = new Trigger(() -> blue3.getAsBoolean());
  public Trigger processorTrigger = new Trigger(() -> blue2.getAsBoolean());

  public RobotContainer() {
    zeroSubsystems.addRequirements(state);

    drivetrain
        .setDefaultCommand(
            new DriveTeleop(this, () -> getController().getLeftY(), () -> getController().getLeftX(),
                () -> getController().getRightX(), slowModeTrigger, leftReefTrigger, rightReefTrigger,
                leftCoralStationTrigger,
                rightCoralStationTrigger, processorTrigger));

    configureController(getController());
    configureButtonBoard(getButtonBoard());
  }

  private void configureController(CommandXboxController joystick) {
    joystick.leftBumper().onTrue(TRY_CORAL_L4).onFalse(TRY_NONE);
    joystick.rightBumper().onTrue(TRY_CORAL_L1).onFalse(TRY_NONE);
  }

  private void configureButtonBoard(Joystick buttonBoard) {

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}