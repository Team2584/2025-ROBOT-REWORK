package frc.robot.commands.zero;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CONSTANTS;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Wrist;

// TODO
public class Zero_Wrist extends Command {
    Wrist wrist;

    public Zero_Wrist(RobotContainer RC) {
        this.wrist = RC.getWrist();

        addRequirements(wrist);
    }

    @Override
    public void initialize() {
        wrist.setWristAngle(CONSTANTS.CONSTANTS_WRIST.PIVOT_DEFAULT);

    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return wrist.isAtSpecificSetpoint(CONSTANTS.CONSTANTS_WRIST.PIVOT_DEFAULT);
    }
}