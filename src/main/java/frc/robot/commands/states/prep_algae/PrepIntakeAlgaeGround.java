package frc.robot.commands.states.prep_algae;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

public class PrepIntakeAlgaeGround extends Command {
    State state;
    Wrist wrist;
    Elevator elevator;
    Algae algae;
    Coral coral;

    public PrepIntakeAlgaeGround(RobotContainer RC) {
        this.state = RC.getState();
        this.wrist = RC.getWrist();
        this.elevator = RC.getElevator();
        this.algae = RC.getAlgae();
        this.coral = RC.getCoral();
    }

    @Override
    public void initialize() {
        state.setRobotState(State.RobotState.PREP_ALGAE_INTAKE_GROUND);
        wrist.setWristAngle(CONSTANTS_WRIST.PIVOT_ALGAE_GROUND);
        elevator.setPosition(CONSTANTS_ELEVATOR.HEIGHT_ALGAE_GROUND);
        algae.setAlgaeIntakeMotor(CONSTANTS_ALGAE.ALGAE_INTAKE_SPEED);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return elevator.isAtSetPoint() && wrist.isAtSetPoint();
    }

}
