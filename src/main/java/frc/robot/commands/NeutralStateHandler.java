package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class NeutralStateHandler extends InstantCommand {
    public NeutralStateHandler(RobotContainer RC) {
        super(() -> {
            if (RC.getAlgae().hasAlgae()){
                new NeutralAlgaeState(RC).schedule();
            } else {
                new NeutralState(RC).schedule();
            }

        });
    }
}
