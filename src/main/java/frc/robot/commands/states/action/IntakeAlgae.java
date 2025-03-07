package frc.robot.commands.states.action;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.CONSTANTS.CONSTANTS_ALGAE;
import frc.robot.CONSTANTS.CONSTANTS_CORAL;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.State;
import frc.robot.subsystems.State.RobotState;

public class IntakeAlgae extends Command {
  Algae algae;
  Elevator elevator;
  State state;
  Coral coral;

  public IntakeAlgae(RobotContainer RC) {
    this.algae = RC.getAlgae();
    this.elevator = RC.getElevator();
    this.state = RC.getState();
    this.coral = RC.getCoral();
  }

  @Override
  public void initialize() {
    Algae.stateRun = true;
    state.setRobotState(RobotState.INTAKE_ALGAE);
    algae.setAlgaeIntakeMotor(CONSTANTS_ALGAE.ALGAE_INTAKE_SPEED);
  }

  @Override
  public void execute() {
    //algae.setAlgaeIntakeMotor(CONSTANTS_ALGAE.ALGAE_INTAKE_SPEED);
    
  }

  @Override
  public void end(boolean interrupted) {
    // Algae.stateRun = false;
    // if (algae.hasAlgae() && !coral.hasCoral()){
    //   state.tryState(RobotState.HAS_ALGAE);
    // } else if  (algae.hasAlgae() && coral.hasCoral()) {
    //   state.tryState(RobotState.HAS_CORAL_AND_ALGAE);
    // } else if (!algae.hasAlgae() && coral.hasCoral()) {
    //   state.tryState(RobotState.HAS_CORAL);
    // } else if (!coral.hasCoral()) {
    //   state.tryState(RobotState.NONE);
    // }
  }

  @Override
  public boolean isFinished() {
    return algae.hasAlgae();
  }
}
