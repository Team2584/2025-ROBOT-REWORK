package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.CONSTANTS;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.CONSTANTS.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class NeutralAlgaeState extends ParallelCommandGroup {
    Elevator elevator;
    Wrist wrist;
    Coral coral;
    Algae algae;
    Ramp ramp;

    public NeutralAlgaeState(RobotContainer RC) {
        elevator = RC.getElevator();
        wrist = RC.getWrist();
        coral = RC.getCoral();
        algae = RC.getAlgae();
        ramp = RC.getRamp();

        addCommands(

                new InstantCommand(() -> algae.setAlgaeIntakeMotor(CONSTANTS_ALGAE.ALGAE_HOLD_SPEED)),

                new InstantCommand(() -> coral.setCoralMotor(0)),

                new InstantCommand(() -> ramp.setNeutral()),

                wrist.setWristAngleCommand(CONSTANTS_WRIST.PIVOT_ALGAE_NEUTRAL)
                        .withTimeout(CONSTANTS_WRIST.WRIST_TIMEOUT),

                new InstantCommand(() -> elevator.setPosition(CONSTANTS_ELEVATOR.ZEROED_POS))
                        .withTimeout(CONSTANTS_ELEVATOR.ELEVATOR_MAX_TIMEOUT)
                        .andThen(new InstantCommand(() -> elevator.homeElevator())));
    

    }

}