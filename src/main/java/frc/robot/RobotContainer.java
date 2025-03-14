package frc.robot;

import java.util.List;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.CONSTANTS.*;
import frc.robot.commands.AddVisionMeasurement;
import frc.robot.commands.DriveTeleop;
import frc.robot.commands.NeutralAlgaeState;
import frc.robot.commands.NeutralState;
import frc.robot.commands.NeutralStateHandler;
import frc.robot.commands.TOFDrive;
import frc.robot.commands.zero.Zero_Elevator;
import frc.robot.commands.zero.Zero_Wrist;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Ramp;
import frc.robot.subsystems.State;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.State.DriverState;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.subsystems.swerve.TunerConstants;
import frc.robot.commands.prep_coral.*;
import frc.robot.commands.prep_algae.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

@Logged
public class RobotContainer {
  @NotLogged
  private final CommandXboxController controller = new CommandXboxController(CONSTANTS_PORTS.CONTROLLER_PORT);
  @NotLogged
  private final Joystick buttonBoard = new Joystick(CONSTANTS_PORTS.BUTTON_BOARD_PORT);

  private final State state = new State(this);
  private final Drivetrain drivetrain = new Drivetrain();
  private final CommandSwerveDrivetrain commandSwerveDrivetrain = TunerConstants.createDrivetrain();
  private final Elevator elevator = new Elevator();
  private final Climber climber = new Climber();
  private final Ramp ramp = new Ramp();
  private final Wrist wrist = new Wrist();
  private final Algae algae = new Algae();
  private final Coral coral = new Coral();
  private final Vision vision = new Vision();

  @NotLogged
  SendableChooser<Command> autoChooser = new SendableChooser<>();

  @NotLogged
  Pose2d[] SELECTED_AUTO_PREP_MAP;
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

  public CommandSwerveDrivetrain getCommandSwerveDrivetrain() {
    return this.commandSwerveDrivetrain;
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

  public Vision getVision() {
    return this.vision;
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
  public Trigger leftReefTrigger = new Trigger(() -> getController().leftBumper().getAsBoolean());
  public Trigger rightReefTrigger = new Trigger(() -> getController().rightBumper().getAsBoolean());

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
    configureAutoBindings();
    configureAutoSelector();

  }

  private void configureController() {
    controller.x().onTrue(new PrepIntakeCoral(this));
    controller.a().whileTrue(coral.outtakeCoral());
    controller.y().whileTrue(algae.outtakeAlgae());

    controller.b().whileTrue(new TOFDrive(this, CONSTANTS_DRIVETRAIN.TOF_SPEED, CONSTANTS_DRIVETRAIN.TOF_DISTANCE)
    .andThen(Commands.runEnd(()->coral.setCoralMotor(CONSTANTS_CORAL.CORAL_OUTTAKE_SPEED), ()->coral.setCoralMotor(0)))
    .until(()->!coral.hasCoral()));
  }

  private void configureButtonBoard() {

    redL4.onTrue(new PrepCoralLvl4(this))
        .onFalse(new NeutralStateHandler(this));

    redL3.onTrue(new PrepCoralLvl3(this))
        .onFalse(new NeutralStateHandler(this));

    redL2.onTrue(new PrepCoralLvl2(this))
        .onFalse(new NeutralStateHandler(this));

    redL1.onTrue(new PrepCoralLvl1(this))
        .onFalse(new NeutralStateHandler(this));

    blue4.onTrue(new PrepNetAlgae(this))
        .onFalse(new NeutralStateHandler(this));

    blue3.onTrue(new PickupReefHighAlgae(this))
        .onFalse(new NeutralStateHandler(this));

    blue2.onTrue(new PickupReefLowAlgae(this))
        .onFalse(new NeutralStateHandler(this));

    blue1.onTrue(new PickupAlgaeGround(this))
        .onFalse(new NeutralStateHandler(this));
  }

  /* AUTO STUFF */

  public Command getAutonomousCommand() {
    selectAutoMap();
    return autoChooser.getSelected();
  }

  public void resetToAutoPose() {
    Rotation2d desiredRotation = Rotation2d.kZero;

    try {
      desiredRotation = PathPlannerAuto.getPathGroupFromAutoFile(autoChooser.getSelected().getName()).get(0)
          .getIdealStartingState().rotation();
      if (CONSTANTS_FIELD.isRedAlliance()) {
        desiredRotation = desiredRotation.plus(Rotation2d.k180deg);
      }
    } catch (Exception e) {
    }

    drivetrain.resetPoseToPose(new Pose2d(drivetrain.getPose().getTranslation(), desiredRotation));
  }

  private void configureAutoSelector() {
    autoChooser = AutoBuilder.buildAutoChooser("4PIECE_L4_HIGH");
    SmartDashboard.putData(autoChooser);
  }

  private void configureAutoBindings() {

    Command driveAutoAlign = Commands.runOnce(() -> drivetrain.autoAlign(Meters.of(0),
        SELECTED_AUTO_PREP_MAP[AUTO_PREP_NUM], MetersPerSecond.of(0),
        MetersPerSecond.of(0), DegreesPerSecond.of(0), 1.0, false, Meters.of(1000), DriverState.REEF_AUTO_DRIVING,
        DriverState.REEF_AUTO_DRIVING, state)).repeatedly();

    NamedCommands.registerCommand("PlaceSequenceL4",
        Commands.sequence(
            driveAutoAlign.asProxy().withTimeout(1), // Attempt to align for up to 1 second
            Commands.runOnce(() -> drivetrain.drive(new ChassisSpeeds(), false)), // Stop driving
            new PrepCoralLvl4(this).asProxy().withTimeout(CONSTANTS_ELEVATOR.ELEVATOR_MAX_TIMEOUT), // Give it time to
                                                                                                    // "prepare" without
                                                                                                    // checking state
            coral.outtakeCoral().asProxy().withTimeout(0.25), // Give it time to "score" without checking state
            Commands.runOnce(() -> AUTO_PREP_NUM++) // Increment counter
        ).withName("PlaceSequence"));

    NamedCommands.registerCommand("autoAlign",
     driveAutoAlign.withTimeout(5));

    NamedCommands.registerCommand("PrepPlace",
        new PrepCoralLvl4(this).withTimeout(CONSTANTS_ELEVATOR.ELEVATOR_MAX_TIMEOUT)
            .asProxy().withName("PrepPlace"));

    NamedCommands.registerCommand("GetCoralStationPiece",
        coral.intakeCoral().asProxy().until(() -> coral.coralLoaded())
            .withName("GetCoralStationPiece"));

    NamedCommands.registerCommand("prepNet", new PrepNetAlgae(this).withTimeout(1));

    NamedCommands.registerCommand("wrist60Deg", wrist.setWristAngleCommand(CONSTANTS_WRIST.PIVOT_ALGAE_NEUTRAL)
        .withTimeout(0.3));

    NamedCommands.registerCommand("shootAlgae", algae.outtakeAlgae());

    NamedCommands.registerCommand("liftLowAlgae", new PickupReefLowAlgae(this).withTimeout(1));

    NamedCommands.registerCommand("algaeNeutral", new NeutralAlgaeState(this).withTimeout(1));

    NamedCommands.registerCommand("neutral", new NeutralState(this).withTimeout(1));

    NamedCommands.registerCommand("liftL4", new PrepCoralLvl4(this).withTimeout(0.5));

    NamedCommands.registerCommand("liftL3", new PrepCoralLvl3(this).withTimeout(0.3));

    NamedCommands.registerCommand("scoreCoral", coral.outtakeCoral().withTimeout(0.125));

    NamedCommands.registerCommand("liftHighAlgae", new PickupReefHighAlgae(this).withTimeout(1.5));

    NamedCommands.registerCommand("liftNet", new PrepNetAlgae(this));

    NamedCommands.registerCommand("scoreNet", algae.outtakeAlgae().withTimeout(0.3));

    NamedCommands.registerCommand("intakeCoral", coral.intakeCoral());

    NamedCommands.registerCommand("TOFDrive&Score", new TOFDrive(this,CONSTANTS_DRIVETRAIN.TOF_SPEED ,CONSTANTS_DRIVETRAIN.TOF_DISTANCE).andThen(coral.outtakeCoral().withTimeout(0.125)));

    // // -- Event Markers --
    EventTrigger prepPlace = new EventTrigger("PrepPlace");
    prepPlace.onTrue(new PrepCoralLvl4(this).withTimeout(CONSTANTS_ELEVATOR.ELEVATOR_MAX_TIMEOUT));

    EventTrigger getCoralStationPiece = new EventTrigger("GetCoralStationPiece");
    getCoralStationPiece.onTrue(coral.intakeCoral());

    EventTrigger neutral = new EventTrigger("neutral");
    neutral.onTrue(new NeutralState(this).withTimeout(1));
  }

  private void selectAutoMap() {
    SELECTED_AUTO_PREP_MAP = configureAutoPrepMaps(autoChooser.getSelected().getName());
    SELECTED_AUTO_PREP_MAP_NAME = autoChooser.getSelected().getName();
  }

  private Pose2d[] configureAutoPrepMaps(String selectedAuto) {
    List<Pose2d> fieldPositions = CONSTANTS_FIELD.getReefPositions().get();

    switch (selectedAuto) {
      case "4PIECE_L4_HIGH":
        Pose2d[] PIECE_L4_HIGH = new Pose2d[4];
        PIECE_L4_HIGH[0] = fieldPositions.get(9); // J
        PIECE_L4_HIGH[1] = fieldPositions.get(10); // K
        PIECE_L4_HIGH[2] = fieldPositions.get(11); // L
        PIECE_L4_HIGH[3] = fieldPositions.get(0); // A
        return PIECE_L4_HIGH;
      case "CenterAuto":
        Pose2d[] CenterAuto = new Pose2d[1];
        CenterAuto[0] = fieldPositions.get(7); // H
       
        return CenterAuto;
      default:
        Pose2d[] noAutoSelected = new Pose2d[1];
        noAutoSelected[0] = new Pose2d();
        return noAutoSelected;
    }
  }

  public void setMegaTag2(boolean setMegaTag2) {

    if (setMegaTag2) {
      drivetrain.swervePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(
          CONSTANTS_VISION.MEGA_TAG2_STD_DEVS_POSITION,
          CONSTANTS_VISION.MEGA_TAG2_STD_DEVS_POSITION,
          CONSTANTS_VISION.MEGA_TAG2_STD_DEVS_HEADING));
    } else {
      // Use MegaTag 1
      drivetrain.swervePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(
          CONSTANTS_VISION.MEGA_TAG1_STD_DEVS_POSITION,
          CONSTANTS_VISION.MEGA_TAG1_STD_DEVS_POSITION,
          CONSTANTS_VISION.MEGA_TAG1_STD_DEVS_HEADING));
    }
    vision.setMegaTag2(setMegaTag2);
  }

  public boolean isAligned() {
    return drivetrain.isAligned();
  }

  public Command AddVisionMeasurement() {
    return new AddVisionMeasurement(this)
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming).ignoringDisable(true);
  }

}