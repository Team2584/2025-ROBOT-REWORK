package frc.robot.commands.states;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CONSTANTS.*;
import frc.robot.subsystems.*;

// TODO: UPDATE FOR ALL SUBSYSTEMS
public class None extends Command {
    State globalState;

    public None(State state) {
        globalState = state;
        addRequirements(globalState);
    }

    @Override
    public void initialize() {
        globalState.setRobotState(State.RobotState.NONE);
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
