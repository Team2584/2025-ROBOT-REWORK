package frc.robot.commands.prep_algae;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.CONSTANTS;
import frc.robot.RobotContainer;
import frc.robot.CONSTANTS.CONSTANTS_ELEVATOR;
import frc.robot.CONSTANTS.CONSTANTS_WRIST;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WrapperCommand;

public class PrepNetAlgae extends ParallelCommandGroup {
    Elevator elevator;
    Wrist wrist;
    Algae algae;

    public PrepNetAlgae(RobotContainer robotContainer) {
        elevator = robotContainer.getElevator();
        wrist = robotContainer.getWrist();
        algae = robotContainer.getAlgae();

        addCommands(

            new InstantCommand(()->wrist.setWristAngle(CONSTANTS_WRIST.PIVOT_ALGAE_NET)),
            
            new InstantCommand(()->elevator.setPosition(CONSTANTS_ELEVATOR.HEIGHT_NET))
                .withTimeout(CONSTANTS_ELEVATOR.ELEVATOR_MAX_TIMEOUT)

        );

    }

}