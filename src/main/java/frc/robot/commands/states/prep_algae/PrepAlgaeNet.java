package frc.robot.commands.states.prep_algae;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CONSTANTS.CONSTANTS_ALGAE;
import frc.robot.CONSTANTS.CONSTANTS_CORAL;
import frc.robot.CONSTANTS.CONSTANTS_ELEVATOR;
import frc.robot.CONSTANTS.CONSTANTS_WRIST;
import frc.robot.CONSTANTS;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.State;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.State.RobotState;

public class PrepAlgaeNet extends Command {
    State state;
    Wrist wrist;
    Elevator elevator;
    Algae algae;
    Coral coral;

    public PrepAlgaeNet(RobotContainer RC) {
        this.state = RC.getState();
        this.wrist = RC.getWrist();
        this.elevator = RC.getElevator();
        this.algae = RC.getAlgae();
        this.coral = RC.getCoral();

        addRequirements(state);
    }

    @Override
    public void initialize() {
        state.setRobotState(State.RobotState.PREP_ALGAE_BARGE);
        wrist.setWristAngle(CONSTANTS_WRIST.PIVOT_ALGAE_NET);
        elevator.setPosition(CONSTANTS_ELEVATOR.HEIGHT_BARGE);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            if (algae.hasAlgae() && coral.hasCoral()) {
                state.setRobotState(RobotState.HAS_CORAL_AND_ALGAE);
            } else if (coral.hasCoral()) {
                state.setRobotState(RobotState.HAS_CORAL);
            } else {
                state.setRobotState(RobotState.NONE);
            }
            state.tryState(RobotState.NONE);
        }
    }

    @Override
    public boolean isFinished() {
        return elevator.isAtSetPoint() && wrist.isAtSetPoint();
    }

}
