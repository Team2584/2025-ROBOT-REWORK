package frc.robot.commands.climbing;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;

public class ClimberWinchIn extends ParallelCommandGroup {

    public ClimberWinchIn(RobotContainer RC) {

        addCommands(
                RC.getRamp().rampUpCMD().until(() -> RC.getClimber().isClimbed()),
                RC.getClimber().liftRobot().until(() -> RC.getClimber().isClimbed()));
    }
}