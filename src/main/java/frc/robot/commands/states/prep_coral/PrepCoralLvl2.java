package frc.robot.commands.states.prep_coral;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.CONSTANTS.CONSTANTS_ELEVATOR;
import frc.robot.CONSTANTS.CONSTANTS_WRIST;
import frc.robot.subsystems.*;

public class PrepCoralLvl2 extends Command {
  State globalState;
  Elevator globalElevator;
  Distance globalDistance;
  Wrist wrist;

  public PrepCoralLvl2(RobotContainer RC) {
    globalState = RC.getState();
    globalElevator = RC.getElevator();
    this.wrist = RC.getWrist();
  }

  @Override
  public void initialize() {
    globalState.setRobotState(State.RobotState.PREP_CORAL_L2);

    globalElevator.setPosition(CONSTANTS_ELEVATOR.HEIGHT_CORAL_L2);
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