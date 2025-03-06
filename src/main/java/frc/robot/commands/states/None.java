package frc.robot.commands.states;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.CONSTANTS.*;
import frc.robot.subsystems.*;

// TODO: UPDATE FOR ALL SUBSYSTEMS
public class None extends Command {
<<<<<<< HEAD
    State state;
    Climber climber;
    Ramp ramp;

    public None(RobotContainer RC) {
        this.state = RC.getState();
        this.climber = RC.getClimber();
        this.ramp = RC.getRamp();

        addRequirements(state);
=======
    State globalState;
    Elevator globalElevator;
    Coral globalCoral;

    public None(RobotContainer RC) {
        globalState = RC.getState();
        globalElevator = RC.getElevator();
        globalCoral = RC.getCoral();
>>>>>>> origin/coralTest
    }

    @Override
    public void initialize() {
<<<<<<< HEAD
        state.setRobotState(State.RobotState.NONE);
        climber.setClimberMotorVelocity(0);
        ramp.setRampMotorVelocity(0);
=======
        globalState.setRobotState(State.RobotState.NONE);
        //globalElevator.setPosition(Units.Inches.of(2));
        globalCoral.setCoralMotor(0);
>>>>>>> origin/coralTest
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
