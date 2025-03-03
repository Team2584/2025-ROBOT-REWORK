package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.Drivetrain;

@Logged
public class State extends SubsystemBase {
  public static DriverState currentDriverState;
  public static RobotState currentRobotState;

  @NotLogged
  RobotContainer RC;
  @NotLogged
  Drivetrain drivetrain;
  @NotLogged
  CommandXboxController controller;
  @NotLogged
  Joystick ButtonBoard;

  /** Creates a new StateMachine. */
  // TODO: ADD NEW SUBSYSTEMS WHEN CREATED!!!!
  public State(RobotContainer RC) {
    currentRobotState = RobotState.NONE;
    currentDriverState = DriverState.MANUAL;

    this.RC = RC;
    this.drivetrain = RC.getDrivetrain();
    this.controller = RC.getController();
  }

  public void setDriverState(DriverState driverState) {
    currentDriverState = driverState;
  }

  public void setRobotState(RobotState robotState) {
    currentRobotState = robotState;
  }

  public DriverState getDriverState() {
    return currentDriverState;
  }

  public RobotState getRobotState() {
    return currentRobotState;
  }

  // TODO: make all the statesssss :)
  public Command tryState(RobotState desiredState) {
    switch (desiredState) {
      case NONE:
        switch (currentRobotState) {

        }
        break;

    }

    return Commands
        .print("You gave it a no good state bud. You tried: " + desiredState.toString()
            + " while at " + currentRobotState.toString() + " and it broke...");
  }

  public static enum DriverState {
    MANUAL,

    REEF_ROTATION_SNAPPING,
    CORAL_STATION_ROTATION_SNAPPING,
    PROCESSOR_ROTATION_SNAPPING,

    REEF_AUTO_DRIVING,
    BARGE_AUTO_DRIVING,
    CORAL_STATION_AUTO_DRIVING,
    PROCESSOR_AUTO_DRIVING,
  }

  /**
   * Represents the various states the robot can be in
   */
  public static enum RobotState {
    NONE,

    // Hold 1 Element
    HAS_CORAL,
    HAS_ALGAE,

    // Hold 2 Elements
    HAS_CORAL_AND_ALGAE,
  }

  @Override
  public void periodic() {

  }
}