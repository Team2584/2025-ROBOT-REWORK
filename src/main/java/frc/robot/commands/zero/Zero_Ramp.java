package frc.robot.commands.zero;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Ramp;

// TODO
public class Zero_Ramp extends Command {
    Ramp ramp;

    public Zero_Ramp(RobotContainer RC) {
        this.ramp = RC.getRamp();

        addRequirements(ramp);
    }

    @Override
    public void initialize() {

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