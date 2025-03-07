package frc.robot.commands.states.climbing;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CONSTANTS;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.State;

public class ClimberDeploying extends Command {
  State state;
  Climber climber;

  public ClimberDeploying(RobotContainer RC) {
    state = RC.getState();
    climber = RC.getClimber();

    addRequirements(state);
  }

  @Override
  public void initialize() {
    climber.setClimberMotorVelocity(CONSTANTS.CONSTANTS_CLIMB.CLIMBER_DEPLOYING_VELOCITY);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    climber.setClimberMotorVelocity(0);
  }

  @Override
  public boolean isFinished() {
    return climber.isClimbDeployed();
  }
}