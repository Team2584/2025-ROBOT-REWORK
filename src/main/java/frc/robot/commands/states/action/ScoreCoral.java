package frc.robot.commands.states.action;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.CONSTANTS.CONSTANTS_CORAL;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.State;

public class ScoreCoral extends Command {
  Coral globalCoral;
  Elevator globalElevator;
  State globalState;
  double coralOuttakeSpeed;

  public ScoreCoral(RobotContainer RC) {
    globalCoral = RC.getCoral();
    globalElevator = RC.getElevator();
    globalState = RC.getState();
  }

  @Override
  public void initialize() {
    globalCoral.setCoralMotor(CONSTANTS_CORAL.CORAL_INTAKE_SPEED);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    globalCoral.setCoralMotor(0);
  }

  @Override
  public boolean isFinished() {
    return !globalCoral.hasCoral();
  }
}
