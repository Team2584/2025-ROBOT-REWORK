package frc.robot.commands.states;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CONSTANTS.*;
import frc.robot.subsystems.*;

// TODO: UPDATE FOR ALL SUBSYSTEMS
public class None extends Command {
    State globalStateMachine;

    public None(State subStateMachine) {
        globalStateMachine = subStateMachine;
        addRequirements(globalStateMachine);
    }

    @Override
    public void initialize() {
        globalStateMachine.setRobotState(State.RobotState.NONE);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
