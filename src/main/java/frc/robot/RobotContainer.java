package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.google.flatbuffers.Constants;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.CONSTANTS.CONSTANTS_DRIVETRAIN;
import frc.robot.CONSTANTS.CONSTANTS_ELEVATOR;
import frc.robot.CONSTANTS.CONSTANTS_PORTS;
import frc.robot.commands.DriveTeleop;
import frc.robot.commands.states.None;
import frc.robot.commands.states.action.IntakeAlgae;
import frc.robot.commands.states.action.ScoreAlgae;
import frc.robot.commands.states.action.ScoreCoral;
import frc.robot.commands.states.hold.HasAlgae;
import frc.robot.commands.states.prep_algae.PrepAlgaeNet;
import frc.robot.commands.states.prep_algae.PrepIntakeAlgaeGround;
import frc.robot.commands.states.prep_algae.PrepIntakeAlgaeHighReef;
import frc.robot.commands.states.prep_algae.PrepIntakeAlgaeLowReef;
import frc.robot.commands.states.prep_coral.PrepCoralLvl;
import frc.robot.commands.states.prep_coral.*;
import frc.robot.commands.zero.Zero_Elevator;
import frc.robot.commands.zero.Zero_Ramp;
import frc.robot.commands.zero.Zero_Wrist;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Coral;
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
  private final Algae algae = new Algae();
  private final Coral coral = new Coral();

  @NotLogged
  SendableChooser<Command> autoChooser = new SendableChooser<>();

  Command TRY_NONE = Commands.deferredProxy(() -> state.tryState(RobotState.NONE));
  Command TRY_CORAL_ZERO = Commands.deferredProxy(() -> state.tryState(RobotState.PREP_CORAL_ZERO));
  Command TRY_CORAL_L1 = Commands.deferredProxy(() -> state.tryState(RobotState.PREP_CORAL_L1));
  Command TRY_CORAL_L2 = Commands.deferredProxy(() -> state.tryState(RobotState.PREP_CORAL_L2));
  Command TRY_CORAL_L3 = Commands.deferredProxy(() -> state.tryState(RobotState.PREP_CORAL_L3));
  Command TRY_CORAL_L4 = Commands.deferredProxy(() -> state.tryState(RobotState.PREP_CORAL_L4));
  Command TRY_SCORE_CORAL = Commands.deferredProxy(() -> state.tryState(RobotState.SCORE_CORAL));
  Command TRY_HAS_CORAL = Commands.deferredProxy(() -> state.tryState(RobotState.HAS_CORAL));
  Command TRY_INTAKE_CORAL = Commands.deferredProxy(() -> state.tryState(RobotState.INTAKE_CORAL));
  Command TRY_PREP_ALGAE_NET = Commands.deferredProxy(() -> state.tryState(RobotState.PREP_ALGAE_BARGE));
  Command TRY_INTAKE_ALGAE = Commands.deferredProxy(() -> state.tryState(RobotState.INTAKE_ALGAE));
  Command TRY_HAS_ALGAE = Commands.deferredProxy(() -> state.tryState(RobotState.HAS_ALGAE));
  Command TRY_PREP_INTAKE_ALGAE_GROUND = Commands
      .deferredProxy(() -> state.tryState(RobotState.PREP_ALGAE_INTAKE_GROUND));
  Command TRY_PREP_INTAKE_ALGAE_HIGH_REEF = Commands
      .deferredProxy(() -> state.tryState(RobotState.PREP_ALGAE_INTAKE_REEF_HIGH));
  Command TRY_PREP_INTAKE_ALGAE_LOW_REEF = Commands
      .deferredProxy(() -> state.tryState(RobotState.PREP_ALGAE_INTAKE_REEF_LOW));
  Command TRY_SCORE_ALGAE = Commands.deferredProxy(() -> state.tryState(RobotState.SCORE_ALGAE));

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

  public Algae getAlgae() {
    return this.algae;
  }

  public Coral getCoral() {
    return this.coral;
  }

  // TODO: add other subsystems to this command
  Command zeroSubsystems = new ParallelCommandGroup(
      new Zero_Elevator(this).withTimeout(CONSTANTS_ELEVATOR.ZEROING_TIMEOUT.in(Units.Seconds)),
      new Zero_Wrist(this))
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
  public Trigger slowModeTrigger = new Trigger(() -> getController().leftTrigger().getAsBoolean());
  public Trigger leftReefTrigger = new Trigger(() -> false);
  public Trigger rightReefTrigger = new Trigger(() -> false);

  public Trigger rightCoralStationTrigger = new Trigger(() -> false);
  public Trigger leftCoralStationTrigger = new Trigger(() -> false);
  public Trigger processorTrigger = new Trigger(() -> false);

  public RobotContainer() {
    zeroSubsystems.addRequirements(state);

    drivetrain
        .setDefaultCommand(
            new DriveTeleop(this, () -> getController().getLeftY(), () -> getController().getLeftX(),
                () -> getController().getRightX(), slowModeTrigger, leftReefTrigger, rightReefTrigger,
                leftCoralStationTrigger,
                rightCoralStationTrigger, processorTrigger));

    configureController();
    configureButtonBoard();
    checkForCoral();
  }

  private void configureController() {
    controller.x().onTrue(TRY_INTAKE_CORAL);
    controller.a().onTrue(TRY_SCORE_CORAL);
    controller.y().onTrue(new ScoreAlgae(this)).onFalse(new None(this));
    controller.b().onTrue(TRY_NONE).onFalse(zeroSubsystems);
  }

  private void configureButtonBoard() {
    redL4.onTrue(new PrepCoralLvl4(this)).onFalse(new None(this));
    redL3.onTrue(new PrepCoralLvl3(this)).onFalse(new None(this));
    redL2.onTrue(new PrepCoralLvl2(this)).onFalse(new None(this));
    redL1.onTrue(new PrepCoralLvl1(this)).onFalse(new None(this));

    blue4.onTrue(new SequentialCommandGroup(new PrepAlgaeNet(this))).onFalse(new None(this));
    blue3.onTrue(new SequentialCommandGroup(new PrepIntakeAlgaeHighReef(this), new IntakeAlgae(this))).onFalse(new HasAlgae(this));
    blue2.onTrue(new SequentialCommandGroup(new PrepIntakeAlgaeLowReef(this), new IntakeAlgae(this))).onFalse(new HasAlgae(this));
    blue1.whileTrue(new SequentialCommandGroup(new PrepIntakeAlgaeGround(this), new IntakeAlgae(this))).onFalse(new HasAlgae(this));
  }

  public void checkForCoral() {
    if (coral.coralLoaded()) {
      Commands.deferredProxy(() -> state.tryState(RobotState.HAS_CORAL));
    }
  }

  public void checkForAlgae() {
    if(algae.hasAlgae()) {
      Commands.deferredProxy(() -> state.tryState(RobotState.HAS_ALGAE));
    }
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  
}