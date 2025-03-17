package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.CONSTANTS.CONSTANTS_ALGAE;

public class NeutralStateHandler extends InstantCommand {
    public NeutralStateHandler(RobotContainer RC) {
        super(() -> {
            RC.getAlgae().setAlgaeIntakeMotor(CONSTANTS_ALGAE.ALGAE_HOLD_SPEED);
            if (RC.getAlgae().hasAlgae()) {
                new NeutralAlgaeState(RC).schedule();
            } else {
                new NeutralState(RC).schedule();
            }

        });
    }
}
