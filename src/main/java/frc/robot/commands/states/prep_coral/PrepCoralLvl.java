package frc.robot.commands.states.prep_coral;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.CONSTANTS.CONSTANTS_ELEVATOR;
import frc.robot.CONSTANTS.CONSTANTS_WRIST;
import frc.robot.subsystems.*;

public class PrepCoralLvl extends Command {
  State globalState;
  Elevator globalElevator;
  Distance globalDistance;
  Wrist wrist;

  public PrepCoralLvl(RobotContainer RC, Distance height) {
    globalState = RC.getState();
    globalElevator = RC.getElevator();
    this.wrist = RC.getWrist();
    globalDistance = height;
  }

  @Override
  public void initialize() {
    if (globalDistance.equals(CONSTANTS_ELEVATOR.HEIGHT_CORAL_L1))
      globalState.setRobotState(State.RobotState.PREP_CORAL_L1);
    else if (globalDistance.equals(CONSTANTS_ELEVATOR.HEIGHT_CORAL_L2))
      globalState.setRobotState(State.RobotState.PREP_CORAL_L2);
    else if (globalDistance.equals(CONSTANTS_ELEVATOR.HEIGHT_CORAL_L3))
      globalState.setRobotState(State.RobotState.PREP_CORAL_L3);
    else if (globalDistance.equals(CONSTANTS_ELEVATOR.HEIGHT_CORAL_L4))
      globalState.setRobotState(State.RobotState.PREP_CORAL_L4);

    globalElevator.setPosition(globalDistance);
    wrist.setWristAngle(CONSTANTS_WRIST.PIVOT_SCORE_CORAL);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return globalElevator.isAtSetPoint();
  }
}