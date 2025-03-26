package frc.robot.commands.prep_coral;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.CONSTANTS.CONSTANTS_ALGAE;
import frc.robot.CONSTANTS.CONSTANTS_ELEVATOR;

public class Score_Coral extends InstantCommand {
    public Score_Coral(RobotContainer RC) {
        super(() -> {
            if (RC.getElevator().isAtSpecificSetpoint(CONSTANTS_ELEVATOR.HEIGHT_CORAL_L4)) {
                RC.getCoral().outtakeCoralL4().schedule();
            } else {
                RC.getCoral().outtakeCoral().schedule();
            }
        });
    }
}
