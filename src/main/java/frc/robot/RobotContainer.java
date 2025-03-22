package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.Joystick;
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
import frc.robot.subsystems.USBCamera;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.commands.prep_coral.*;
import frc.robot.commands.prep_algae.*;

@Logged
public class RobotContainer {
  @NotLogged
  private final CommandXboxController controller = new CommandXboxController(CONSTANTS_PORTS.CONTROLLER_PORT);
  @NotLogged
  private final CommandXboxController controller2 = new CommandXboxController(CONSTANTS_PORTS.CONTROLLER2_PORT);
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
  private final Vision vision = new Vision();
  @NotLogged
  private final Autons autos;
  private final USBCamera climbCamera = new USBCamera();

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

  public Vision getVision() {
    return this.vision;
  }

  public USBCamera getClimbCamera() {
    return this.climbCamera;
  }

  Command zeroSubsystems = new ParallelCommandGroup(
      new Zero_Elevator(this),
      new Zero_Wrist(this));

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
    Autons.configurePPCommands();

    drivetrain
        .setDefaultCommand(
            new DriveTeleop(this, () -> getController().getLeftY(), () -> getController().getLeftX(),
                () -> getController().getRightX(), slowModeTrigger, leftReefTrigger, rightReefTrigger,
                leftCoralStationTrigger,
                rightCoralStationTrigger, processorTrigger));
    autos = new Autons(this);
    configureController();
    configureController2();
    //configureButtonBoard();
  }

// Button Board Controls
/*
  private void configureController() {
    controller.x().onTrue(new PrepIntakeCoral(this));
    controller.a().whileTrue(new InstantCommand (() -> {
                            if (coral.coralLoaded()){
                              new PrepCoralLock(this).schedule();
                            }}).withTimeout(0.15).andThen(coral.outtakeCoral()));

    controller.y().whileTrue(algae.outtakeAlgae());

    redL4.and(controller.b())
        .whileTrue(new TOFDrive(this, CONSTANTS_DRIVETRAIN.TOF_SPEED, CONSTANTS_DRIVETRAIN.TOF_DISTANCE)
            .andThen(Commands.runEnd(() -> coral.setCoralMotor(CONSTANTS_CORAL.CORAL_OUTTAKE_SPEED),
                () -> coral.setCoralMotor(0)))
            .until(() -> !coral.hasCoral()));

    redL3.and(controller.b())
        .whileTrue(new TOFDrive(this, CONSTANTS_DRIVETRAIN.TOF_SPEED, CONSTANTS_DRIVETRAIN.TOF_DISTANCE_LOW)
            .andThen(Commands.runEnd(() -> coral.setCoralMotor(CONSTANTS_CORAL.CORAL_OUTTAKE_SPEED),
                () -> coral.setCoralMotor(0)))
            .until(() -> !coral.hasCoral()));

    redL2.and(controller.b())
        .whileTrue(new TOFDrive(this, CONSTANTS_DRIVETRAIN.TOF_SPEED, CONSTANTS_DRIVETRAIN.TOF_DISTANCE_LOW)
            .andThen(Commands.runEnd(() -> coral.setCoralMotor(CONSTANTS_CORAL.CORAL_OUTTAKE_SPEED),
                () -> coral.setCoralMotor(0)))
            .until(() -> !coral.hasCoral()));

    controller.back().whileTrue(climber.liftRobot().until(() -> climber.isClimbed())); // Lift Robot (Winch in)

    controller.start()
        .whileTrue(new ParallelCommandGroup(
            new InstantCommand(() -> ramp.setRampMotorVelocity(CONSTANTS_RAMP.RAMP_UP_VELOCITY)), climber.lowerRobot()))
        .onFalse(new InstantCommand(() -> ramp.setRampMotorVelocity(CONSTANTS_RAMP.RAMP_UP_VELOCITY / 2))); // Ramp

    controller.leftTrigger().and(controller.rightTrigger()).and(controller.povRight())
        .onTrue(new InstantCommand(() -> MongolianSuperServerBailoutIMU()));
  }
  */


  private void configureButtonBoard() {

    redL4.onTrue(new PrepCoralLvl4(this))
        .onFalse(new NeutralStateHandler(this));

    redL3.onTrue(new PrepCoralLvl3(this))
        .onFalse(new NeutralStateHandler(this));

    redL2.onTrue(new PrepCoralLvl2(this))
        .onFalse(new NeutralStateHandler(this));

    redL1.onTrue(new PrepCoralLvl1(this))
        .onFalse(new NeutralStateHandler(this));

    blue4.whileTrue(new PrepNetAlgae(this))
        .onFalse(new NeutralStateHandler(this));

    blue3.whileTrue(new PickupReefHighAlgae(this))
        .onFalse(new NeutralStateHandler(this));

    blue2.whileTrue(new PickupReefLowAlgae(this))
        .onFalse(new NeutralStateHandler(this));

    blue1.whileTrue(new PickupAlgaeGround(this))
        .onFalse(new NeutralStateHandler(this));
  }

  // Twin Xbox Controlls

  private void configureController(){
    controller.start().and(controller.back())
    .onTrue(new InstantCommand(()-> MongolianSuperServerBailoutIMU()));

    controller.rightTrigger().whileTrue(new PickupAlgaeGround(this))
    .onFalse(new NeutralStateHandler(this));

    controller.b().whileTrue(new PrepProcessorAlgae(this))
    .onFalse(new NeutralStateHandler(this));

    controller.povUp()
    .whileTrue(climber.liftRobot().until(() -> climber.isClimbed())); // Lift Robot (Winch in)

    controller.povDown()
    .whileTrue(new ParallelCommandGroup(new InstantCommand(() -> ramp.setRampMotorVelocity(CONSTANTS_RAMP.RAMP_UP_VELOCITY)), climber.lowerRobot()))
    .onFalse(new InstantCommand(() -> ramp.setRampMotorVelocity(CONSTANTS_RAMP.RAMP_UP_VELOCITY / 2))); // Ramp

  }

  private void configureController2(){

    controller2.leftBumper().whileTrue(new PrepNetAlgae(this))
    .onFalse(new NeutralStateHandler(this));
    
    controller2.leftTrigger().whileTrue(algae.outtakeAlgae());

    controller2.rightTrigger().whileTrue(new InstantCommand (() -> {
      if (coral.coralLoaded()){
        new PrepCoralLock(this).schedule();
      }}).withTimeout(0.15).andThen(coral.outtakeCoral()));

    controller2.rightBumper().onTrue(new PrepIntakeCoral(this));


    controller2.x().onTrue(new PrepCoralLvl1(this))
    .onFalse(new NeutralStateHandler(this));
    
    controller2.y().onTrue(new PrepCoralLvl4(this))
    .onFalse(new NeutralStateHandler(this));

    controller2.b().onTrue(new PrepCoralLvl3(this))
    .onFalse(new NeutralStateHandler(this));

    controller2.a().onTrue(new PrepCoralLvl2(this))
    .onFalse(new NeutralStateHandler(this));

    controller2.povUp().whileTrue(new PickupReefHighAlgae(this))
    .onFalse(new NeutralStateHandler(this));

    controller2.povDown().whileTrue(new PickupReefLowAlgae(this))
    .onFalse(new NeutralStateHandler(this));


  }
  

  /*
   * Make sure the robot is facing YOU (THE DRIVER STATION; AWAY FROM OPPOSING
   * ALLIANCE)
   */
  public void MongolianSuperServerBailoutIMU() {
    if (CONSTANTS_FIELD.isRedAlliance()) {
      drivetrain.resetYaw(0);
    } else if (!CONSTANTS_FIELD.isRedAlliance()) {
      drivetrain.resetYaw(180);

    }
  }
  /* AUTO STUFF */

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