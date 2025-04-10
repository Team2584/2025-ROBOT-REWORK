package frc.robot.commands.prep_coral;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.CONSTANTS.CONSTANTS_ELEVATOR;

public class ScoreCoral extends InstantCommand {
    public ScoreCoral(RobotContainer RC) {
        super(() -> {
            if (RC.getElevator().isAtSpecificSetpoint(CONSTANTS_ELEVATOR.HEIGHT_CORAL_L4)) {
                RC.getCoral().outtakeCoralL4().schedule();
            } else if (RC.getElevator().isAtSpecificSetpoint(CONSTANTS_ELEVATOR.HEIGHT_CORAL_L3)) {
                RC.getCoral().outtakeCoralL3().schedule();
            } else if (RC.getElevator().isAtSpecificSetpoint(CONSTANTS_ELEVATOR.HEIGHT_CORAL_L2)) {
                RC.getCoral().outtakeCoralL2().schedule();
            } else if (RC.getElevator().isAtSpecificSetpoint(CONSTANTS_ELEVATOR.HEIGHT_CORAL_L1)) {
                RC.getCoral().outtakeCoralL1().schedule();
            } else {
                RC.getCoral().outtakeCoralL3().schedule();
            }
            RC.getLED().setColor(255, 0, 0);
        });
    }
}
