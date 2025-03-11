package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.CONSTANTS.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class NeutralState extends ParallelCommandGroup {
        Elevator elevator;
        Wrist wrist;
        Coral coral;
        Algae algae;
        Ramp ramp;

        public NeutralState(RobotContainer RC) {
                elevator = RC.getElevator();
                wrist = RC.getWrist();
                coral = RC.getCoral();
                algae = RC.getAlgae();
                ramp = RC.getRamp();

                addCommands(

                                new InstantCommand(() -> algae.setAlgaeIntakeMotor(CONSTANTS_ALGAE.ALGAE_IDLE_SPEED)),

                                new InstantCommand(() -> coral.setCoralMotor(0)),

                                new InstantCommand(() -> ramp.setNeutral()),

                                wrist.setWristAngleCommand(CONSTANTS_WRIST.PIVOT_DEFAULT)
                                                .withTimeout(CONSTANTS_WRIST.WRIST_TIMEOUT),

                                new InstantCommand(() -> elevator.setPosition(CONSTANTS_ELEVATOR.ZEROED_POS))
                                                .withTimeout(CONSTANTS_ELEVATOR.ELEVATOR_MAX_TIMEOUT)
                                                .andThen(new InstantCommand(() -> elevator.homeElevator())));

        }

}