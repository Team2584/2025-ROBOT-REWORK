package frc.robot.commands.states.action;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CONSTANTS.CONSTANTS_CORAL;
import frc.robot.CONSTANTS.CONSTANTS_RAMP;
import frc.robot.CONSTANTS.CONSTANTS_WRIST;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Ramp;
import frc.robot.subsystems.State;
import frc.robot.subsystems.Wrist;

public class IntakeCoral extends Command {
    State globalState;
    Coral globalCoral;
    Wrist globalWrist;
    Ramp ramp;

    public IntakeCoral(RobotContainer RC) {
        globalState = RC.getState();
        globalCoral = RC.getCoral();
        this.ramp = RC.getRamp();
        globalWrist = RC.getWrist();
    }

    @Override
    public void initialize() {
        globalCoral.setCoralMotor(CONSTANTS_CORAL.CORAL_INTAKE_SPEED);
        ramp.setRampMotorVelocity(CONSTANTS_RAMP.RAMP_INTAKE_VELOCITY);
        globalWrist.setWristAngle(CONSTANTS_WRIST.PIVOT_INTAKE_CORAL);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        globalCoral.setCoralMotor(0);
        ramp.setRampMotorVelocity(0);
    }

    @Override
    public boolean isFinished() {
        return globalCoral.coralLoaded();
    }

}
