package frc.robot.commands.states;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.CONSTANTS.*;
import frc.robot.subsystems.*;

// TODO: UPDATE FOR ALL SUBSYSTEMS
public class None extends Command {
    State globalState;
    Elevator globalElevator;
    Coral globalCoral;

    public None(RobotContainer RC) {
        globalState = RC.getState();
        globalElevator = RC.getElevator();
        globalCoral = RC.getCoral();
    }

    @Override
    public void initialize() {
        globalState.setRobotState(State.RobotState.NONE);
        globalElevator.setPosition(Units.Inches.of(2));
        globalCoral.setCoralOuttake(0);
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
