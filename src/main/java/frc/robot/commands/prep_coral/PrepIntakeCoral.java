package frc.robot.commands.prep_coral;

import frc.robot.CONSTANTS.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.Wrist;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Ramp;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PrepIntakeCoral extends SequentialCommandGroup {
    Elevator elevator;
    Wrist wrist;
    Coral coral;
    Ramp ramp;

    public PrepIntakeCoral(RobotContainer RC) {
        elevator = RC.getElevator();
        wrist = RC.getWrist();
        coral = RC.getCoral();
        ramp = RC.getRamp();

        addCommands(

                new ParallelCommandGroup(
                        new InstantCommand(() -> elevator.setPosition(CONSTANTS_ELEVATOR.ZEROED_POS)),
                        coral.intakeCoral(),
                        new InstantCommand(() -> ramp.setRampMotorVelocity(CONSTANTS_RAMP.RAMP_INTAKE_VELOCITY)),
                        wrist.setWristAngleCommand(CONSTANTS_WRIST.PIVOT_INTAKE_CORAL)).until(() -> coral.coralLoaded())
                        .andThen(new InstantCommand(() -> ramp.setRampMotorVelocity(0)))

        );

    }

}