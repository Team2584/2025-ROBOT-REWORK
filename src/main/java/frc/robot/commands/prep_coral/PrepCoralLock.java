package frc.robot.commands.prep_coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.CONSTANTS.CONSTANTS_CORAL;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.LED;

public class PrepCoralLock extends Command {
    Coral coral;
    LED led;

    public PrepCoralLock(RobotContainer RC) {
        coral = RC.getCoral();
        led = RC.getLED();

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
        if (!interrupted) {
            led.setColor(204, 57, 123);
        }
    }

    @Override
    public boolean isFinished() {
        return coral.coralCleared();
    }
}