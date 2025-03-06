package frc.robot.commands.states.hold;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;
import frc.robot.subsystems.State.RobotState;

public class HasCoral extends Command {
  State globalState;
  Coral globalCoral;
  Elevator globalElevator;

  public HasCoral(RobotContainer RC) {
    globalState = RC.getState();
    globalCoral = RC.getCoral();
    globalElevator = RC.getElevator();
    addRequirements(globalState);
  }

  @Override
  public void initialize() {
    globalState.setRobotState(RobotState.HAS_CORAL);
    globalElevator.setPosition(Units.Inches.zero());
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}