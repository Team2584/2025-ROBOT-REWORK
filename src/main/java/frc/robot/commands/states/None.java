package frc.robot.commands.states;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.CONSTANTS.*;
import frc.robot.subsystems.*;

// TODO: UPDATE FOR ALL SUBSYSTEMS
public class None extends Command {
    State state;
    Climber climber;
    Ramp ramp;
    Elevator elevator;
    Coral coral;

    public None(RobotContainer RC) {
        this.state = RC.getState();
        this.climber = RC.getClimber();
        this.ramp = RC.getRamp();
        this.elevator = RC.getElevator();
        this.coral = RC.getCoral();

        addRequirements(state);
    }

    @Override
    public void initialize() {
        state.setRobotState(State.RobotState.NONE);
        climber.setClimberMotorVelocity(0);
        ramp.setRampMotorVelocity(0);
        coral.setCoralMotor(0);
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
