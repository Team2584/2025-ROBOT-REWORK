package frc.robot.commands.states.action;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CONSTANTS.CONSTANTS_CORAL;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.State;
import frc.robot.subsystems.State.RobotState;

public class IntakeCoral extends Command {
    State globalState;
    Coral globalCoral;

    public IntakeCoral(RobotContainer RC) {
        globalState = RC.getState();
        globalCoral = RC.getCoral();
    }

    public void initialize() {
        globalState.setRobotState(RobotState.INTAKE_CORAL);
        globalCoral.setCoralOuttake(CONSTANTS_CORAL.CORAL_INTAKE_SPEED);
    }

    public void execute() {
    }

    public void end(boolean interrupted) {
        globalCoral.setCoralOuttake(0);
        globalState.tryState(RobotState.HAS_CORAL);
    }

    public boolean isFinished() {
        return globalCoral.coralLoaded();
    }
    
}
