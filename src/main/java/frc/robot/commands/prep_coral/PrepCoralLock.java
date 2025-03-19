package frc.robot.commands.prep_coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.CONSTANTS.CONSTANTS_CORAL;
import frc.robot.subsystems.Coral;

public class PrepCoralLock extends Command {
    Coral coral;

    public PrepCoralLock(RobotContainer RC) {
        coral = RC.getCoral();

        addRequirements(coral);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        coral.setCoralMotor(CONSTANTS_CORAL.CORAL_REV_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        coral.setCoralMotor(0);
    }

    @Override
    public boolean isFinished() {
        return coral.coralCleared();
    }
}