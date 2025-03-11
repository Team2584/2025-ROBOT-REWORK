package frc.robot.commands.prep_coral;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.*;
import frc.robot.CONSTANTS.*;
import frc.robot.RobotContainer;

public class PrepCoralLvl4 extends SequentialCommandGroup {
  Elevator elevator;
  Wrist wrist;
  Coral coral;

  public PrepCoralLvl4(RobotContainer RC) {
    elevator = RC.getElevator();
    wrist = RC.getWrist();
    coral = RC.getCoral();

    addRequirements(elevator, wrist, coral);

    addCommands(

        new InstantCommand(() -> elevator.setPosition(CONSTANTS_ELEVATOR.HEIGHT_CORAL_L4))
            .withTimeout(CONSTANTS_ELEVATOR.ELEVATOR_MAX_TIMEOUT),

        new InstantCommand(() -> wrist.setWristAngle(CONSTANTS_WRIST.PIVOT_SCORE_CORAL))

    );

  }

}