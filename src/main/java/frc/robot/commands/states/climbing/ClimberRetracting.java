package frc.robot.commands.states.climbing;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CONSTANTS;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Ramp;
import frc.robot.subsystems.State;

public class ClimberRetracting extends Command {
    State state;
    Climber climber;
    Ramp ramp;
    Elevator elevator;
    // TODO: add elevator zero

    public ClimberRetracting(RobotContainer RC) {
        this.state = RC.getState();
        this.climber = RC.getClimber();
        this.ramp = RC.getRamp();
        this.elevator = RC.getElevator();

        addRequirements(state);
    }

    @Override
    public void initialize() {
        climber.setClimberMotorVelocity(CONSTANTS.CONSTANTS_CLIMB.CLIMBER_RETRACT_VELOCITY);
        ramp.setRampMotorVelocity(CONSTANTS.CONSTANTS_RAMP.RAMP_UP_VELOCITY);
    }

    @Override
    public void execute() {
        if (ramp.isRampUp()) {
            ramp.setRampMotorVelocity(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        climber.setClimberMotorVelocity(0);
        ramp.setRampMotorVelocity(0);
    }

    @Override
    public boolean isFinished() {
        return climber.isClimbRetracted();
    }
}