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

public class PrepIntakeCoralAutoSecondary extends SequentialCommandGroup {
    Elevator elevator;
    Wrist wrist;
    Coral coral;
    Ramp ramp;

    public PrepIntakeCoralAutoSecondary(RobotContainer RC) {
        elevator = RC.getElevator();
        wrist = RC.getWrist();
        coral = RC.getCoral();
        ramp = RC.getRamp();

        addCommands(
                new ParallelCommandGroup(
                        coral.intakeCoralModSecondary(),
                        wrist.setWristAngleCommand(CONSTANTS_WRIST.PIVOT_INTAKE_CORAL))
                        .until(() -> coral.coralLoaded())
                        .andThen(new InstantCommand(() -> ramp.setRampMotorVelocity(0))),
                coral.intakeCoralSlowSecondary()

        );

    }

}