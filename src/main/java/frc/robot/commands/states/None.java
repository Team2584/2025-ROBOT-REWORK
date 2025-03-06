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
    Wrist wrist;
    Algae algae;

    public None(RobotContainer RC) {
        this.state = RC.getState();
        this.climber = RC.getClimber();
        this.ramp = RC.getRamp();
        this.elevator = RC.getElevator();
        this.coral = RC.getCoral();
        this.wrist = RC.getWrist();
        this.algae = RC.getAlgae();

        addRequirements(state);
    }

    @Override
    public void initialize() {
        state.setRobotState(State.RobotState.NONE);
        elevator.setPosition(CONSTANTS_ELEVATOR.ELEVATOR_MIN_HEIGHT);
        wrist.setWristAngle(CONSTANTS_WRIST.PIVOT_DEFAULT);
        climber.setClimberMotorVelocity(0);
        ramp.setRampMotorVelocity(0);
        coral.setCoralMotor(0);
        algae.setAlgaeIntakeMotor(CONSTANTS_ALGAE.ALGAE_IDLE_SPEED);
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
