package frc.robot.commands.prep_algae;

import java.util.function.BooleanSupplier;

import frc.robot.RobotContainer;
import frc.robot.subsystems.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.robot.CONSTANTS.*;

public class PickupReefLowAlgae extends ParallelCommandGroup {
  Algae algae;
  Elevator elevator;
  Wrist wrist;

  public PickupReefLowAlgae(RobotContainer RC) {
    algae = RC.getAlgae();
    elevator = RC.getElevator();
    wrist = RC.getWrist();

    addRequirements();

    addCommands(

        new InstantCommand(() -> wrist.setWristAngle(CONSTANTS_WRIST.PIVOT_ALGAE_REEF))
            .withTimeout(CONSTANTS_WRIST.WRIST_TIMEOUT),
        new InstantCommand(() -> elevator.setPosition(CONSTANTS_ELEVATOR.HEIGHT_ALGAE_LOW)),
        algae.intakeAlgae());

  }

}